#include "def.h"
#include "core-maint.h"
#include <algorithm>

//for counting 
extern int cnt_Vs; // count of size V*
extern int cnt_Vp; // count of size V+
extern int cnt_S;  // count the number of all visited edges
extern int cnt_tag;

extern int cnt_edge; // for debug
extern int cnt_edge2; // for debug

namespace yan {

/********SeqCM************************************/
SeqCM::CoreMaint::CoreMaint(size_t n, graph_t &graph, vector<core_t> &core):
        n{n}, graph{graph}, core{core}, PQ(n, V) {
    
    const size_t reserve_size = n; //2^15
    Vblack.reserve(reserve_size);
#ifdef WITH_V_BLACK2GRAY 
    Vblack2gray.reserve(n);
#endif
    Vcolor.reserve(reserve_size);
    adj.resize(n);
    
    R = QUEUE(reserve_size); //2^14

    /* init the Order List by k-order odv */
    max_core = 0;
    for(core_t k: core) {  // find max core
        if (k > max_core) max_core = k;
    }

    max_core += MAX_INCREASE_CORE; // the maximum core number after increasing. 

    Node node;
    V = vector<Node>(n + 2*max_core + 2, node); // Vertices
    H = vector<node_t>(max_core + 1); // head of order list k
    T = vector<node_t>(max_core + 1); // tail of order list k 
}



void SeqCM::CoreMaint::Init(vector<int> &odv) {
    /* head include a empty node to avoid empty check.
    * tail the same as head. 
    * head -> v -> tail*/
    for (size_t k = 0; k < H.size(); k++) { // init order list
        H[k] = n + 2*k;
        T[k] = n + 2*k + 1; // head -> tail: two nodes initially.
        V[H[k]].next = T[k]; V[H[k]].pre = NONE;
        V[T[k]].pre = H[k];  V[T[k]].next = NONE;
    }

    /* link all nodes by the k-order (odv) with head and tail ???*/
    for (node_t v: odv) { 
        core_t k = core[v]; 
        node_t &p = V[T[k]].pre; // insert after p 
        V[p].next = v; 
        V[v].pre = p; V[v].next = T[k];
        V[T[k]].pre = v;
    }

    /*assign the tag for all k list, head.tag = 0, tail = maxtag*/
    for (size_t k = 0; k < H.size(); k++) { 
        node_t head = H[k]; node_t tail = T[k];
        tag_t tag = 0;
        //tag increasing by gap starting with 0 
        for (node_t v= head; v != tail; v=V[v].next) { // head.tag = 0
            V[v].tag = tag;
            tag += INIT_TAG_GAP;
        }      
        V[tail].tag = MAX_TAG; // tail.tag = max tag
    }

#if 1
    /*init mcd, degout is initialized on ComputeCore*/
    for (size_t u = 0; u < n; u++) {
        //deg_t degout = 0;
        deg_t mcd = 0;
        for (node_t v: graph[u]) {
            //if (Order(u, v)) degout++;
            if (core[u] <= core[v]) mcd++;
        }
        //V[u].degout = degout;
        V[u].mcd = mcd;
    }
#else 
    /*init the degout and mcd*/
    for (size_t u = 0; u < n; u++) {
        deg_t degout = 0;
        deg_t mcd = 0;
        for (node_t v: graph[u]) {
            if (Order(u, v)) degout++;
            if (core[u] <= core[v]) mcd++;
        }
        V[u].degout = degout;
        V[u].mcd = mcd;
    }
#endif 

    for(int u = 0; u < n; u++) {
        for(int v : graph[u]) {
            if(Order(u, v)) {
                //add_left_right_edge(u, v);
                if (core[u] < core[v]) {
                    adj[u].k_more.push_back(Edge{v, adj[v].k_less.size()});
                    adj[v].k_less.push_back(Edge{u, adj[u].k_more.size() - 1});
                } else {
                    // assert(SameCoreOrder(u, v))
                    adj[u].k_equal_korder_more.push_back(Edge{v, adj[v].k_equal_korder_less.size()});
                    adj[v].k_equal_korder_less.push_back(Edge{u, adj[u].k_equal_korder_more.size() - 1});
                    adj[v].k_equal_korder_less_to_tmp.push_back(-1);
                }
            }
        }
    }
    // TODO: REMOVE
    for(int u = 0; u < n; u++) verify_adj(u, true);
}

void SeqCM::CoreMaint::add_left_right_edge(node_t u, node_t v) {
    //assert(Order(u, v));
    if (core[u] < core[v]) {
        adj[u].k_more.push_back(Edge{v, adj[v].k_less.size()});
        adj[v].k_less.push_back(Edge{u, adj[u].k_more.size() - 1});
    } else {
        // assert(SameCoreOrder(u, v))
        adj[u].k_equal_korder_more.push_back(Edge{v, adj[v].k_equal_korder_less.size()});
        adj[v].k_equal_korder_less.push_back(Edge{u, adj[u].k_equal_korder_more.size() - 1});
        adj[v].k_equal_korder_less_to_tmp.push_back(-1);
    }
    verify_adj(u); verify_adj(v);
}

void SeqCM::CoreMaint::ComputeCore(std::vector<std::vector<int>>& graph, 
                        std::vector<int>& core, std::vector<int>& order_v, bool with_init) {
  // compute the cores
  auto& deg = core;
  int max_deg = 0;
  int n_ = n;
  for (int i = 0; i < n_; ++i) {
    deg[i] = graph[i].size();
    if (deg[i] > max_deg) {
      max_deg = deg[i];
    }
  }
  std::vector<int> bin(max_deg + 1, 0);
  for (int i = 0; i < n_; ++i) {
    ++bin[deg[i]];
  }
  int start = 0;
  for (int i = 0; i <= max_deg; ++i) {
    int temp = bin[i];
    bin[i] = start;
    start += temp;
  }
  std::vector<int> vert(n_);
  std::vector<int> pos(n_);
  for (int i = 0; i < n_; ++i) {
    pos[i] = bin[deg[i]];
    vert[pos[i]] = i;
    ++bin[deg[i]];
  }
  for (int i = max_deg; i > 0; --i) {
    bin[i] = bin[i-1];
  }
  bin[0] = 0;
  int k = 0;
  auto vis = std::vector<bool>(n_, false);
  for (int i = 0; i < n_; ++i) {
    const int v = vert[i];
    if (deg[v] > k) k = deg[v];
    ASSERT(bin[deg[v]] == i);
    ++bin[deg[v]];
    core[v] = k; // get key
    
    //init k-order
    if(with_init) {order_v[i] = v;} // get k-order O

    vis[v] = true;
    int rem = 0;
    for (const int u : graph[v]) {
        if (vis[u]) continue;
        ++rem;
        const int pw = bin[deg[u]];
        const int pu = pos[u];
        if (pw != pu) {
            const int w = vert[pw];
            vert[pu] = w;
            pos[w] = pu;
            vert[pw] = u;
            pos[u] = pw;
        }
        ++bin[deg[u]];
        --deg[u];
        if (pos[u] == i + 1) {
            bin[deg[u]] = pos[u];
        }
    }
    /*init degout, here is more efficient that check the order of u and v*/
    if(with_init) {V[v].degout = rem;}
  }
}


inline void SeqCM::CoreMaint::ListDelete(node_t x) {
    node_t pre = V[x].pre; node_t next = V[x].next;
    V[pre].next = next; V[next].pre = pre;
    V[x].pre = V[x].next = NONE;
}

/*insert y after x in list*/
inline void SeqCM::CoreMaint::ListInsert(node_t x, node_t y) {
    node_t next = V[x].next;
    V[x].next = y; V[y].pre = x;
    V[y].next = next; V[next].pre = y;
}

/*insert all y after x in list*/
inline void SeqCM::CoreMaint::MultiListInsert(node_t x, vector<node_t> &y) {
    // link all y , scan 0 to size-2
    size_t size = y.size();
    if (1 == size) return ListInsert(x, y[0]);

    for (size_t i = 0; i < size - 1; i++) {  // like the vector
        V[y[i]].next = y[i+1];
        V[y[i+1]].pre = y[i];
    }

    node_t next = V[x].next;
    V[x].next = y[0]; V[y[0]].pre = x;
    V[y[size-1]].next = next; V[next].pre = y[size-1];
}

inline bool SeqCM::CoreMaint::Order(node_t x, node_t y) {
    if ( unlikely(core[x] < core[y]) ) { 
        return true;
    } else if ( likely(core[x] == core[y]) ) {
#ifdef WITH_SUBTAG 
        if (likely(V[x].tag  != V[y].tag))
            return V[x].tag < V[y].tag;
        else 
            return V[x].subtag < V[y].subtag;
#else //without subtag
        return V[x].tag < V[y].tag;
#endif
    } else return false;
}

bool SeqCM::CoreMaint::SameCoreOrder(node_t x, node_t y) {
    if (likely(core[x] == core[y])) {
#ifdef WITH_SUBTAG
        if (likely(V[x].tag  != V[y].tag))
            return V[x].tag < V[y].tag;
        else 
            return V[x].subtag < V[y].subtag;
        
#else 
        return (V[x].tag < V[y].tag);
#endif
    } else return false;
}

/*insert y after x. x->z 
* each list has head and tail as empty nodes. 
* j = 1
* while wj <= j*j :
*   update wj
*   j++  
* relabel k=1...j-1 records 
* 
* with subtag for the O(1) running time. 
* for same tag, can contain the subtag 32 in the worst case. 
*/
int SeqCM::CoreMaint::OrderInsert(node_t x, node_t y) {
#ifndef WITH_SUBTAG
    node_t z = V[x].next; // x-> z 
    unsigned int j = 1;
    const tag_t tag0 = V[x].tag;
    tag_t w = V[z].tag - tag0; // w is tag gap
    
    
    if (unlikely(w <= 1)) {// relabel happens
        //the insertion walks down the list until w > j^2
        while (w <= j * j) { // could be geometric increasing. ???
            z = V[z].next;
            w = V[z].tag - tag0;
            j++;
        }

        // in case x is the tail, scope the tag.
        if (w > INIT_TAG_GAP * j) w = INIT_TAG_GAP * j; 

        //relabel the j-1 record s_1 to s_j-1 with the labels 
        tag_t tagbase = w/j;
        
        for (size_t k = 1, z = V[x].next; k < j; k++, z=V[z].next) {
            // the wide gap w is splited into j parts by w/j, 
            // and each time one part for a tag
            V[z].tag = tagbase * k  + tag0; cnt_tag++;
            
            // YANCOUTO: Aha, this is an extra add to the PQ.
            /************************ update PQ when tag changed********************/
            //if (V[z].inQ) {PQ.push(DATA(z, V[z].tag));}
        }

        //reset w after relabel
        z = V[x].next;
        w = V[z].tag - V[x].tag; 
    }
    
    // in case x is the tail, scope the tag.
    if (w > INIT_TAG_GAP) w = INIT_TAG_GAP; 

    V[y].tag = (V[x].tag + w / 2); cnt_tag++;

    /************************ update PQ when tag changed********************/
    //if (V[y].inQ) {PQ.push(DATA(y, V[y].tag));} 

    ListInsert(x, y);
    return 0;

#else // with subtag. insert y after x. 
    
    node_t z = V[x].next; // x-> z 
    unsigned int j = 1;
    const tag_t tag0 = V[x].tag;
    tag_t w = V[z].tag - tag0; // w is tag gap
    

    if (unlikely(w <= 1)) {// can't find gap
        const subtag_t subtag0 = V[x].subtag;
        subtag_t subw;
        if (w == 1) subw = MAX_SUBTAG - subtag0;
        else subw = V[z].subtag - subtag0; //w == 0
        if (subw > 1) { // use the subtag
            V[y].tag =  V[x].tag;
            V[y].subtag = subtag0 + subw / 2; cnt_tag++;
            goto END;
        } 

        //relabel happens
        //the insertion walks down the list until w > j^2
        while (w <= j * j) { // could be geometric increasing. ???
            z = V[z].next;
            w = V[z].tag - tag0;
            j++;
        }

        // in case x is the tail, scope the tag.
        if (w > INIT_TAG_GAP * j) w = INIT_TAG_GAP * j; 

        //relabel the j-1 record s_1 to s_j-1 with the labels 
        tag_t tagbase = w/j;
        
        for (size_t k = 1, z = V[x].next; k < j; k++, z=V[z].next) {
            // the wide gap w is splited into j parts by w/j, 
            // and each time one part for a tag
            V[z].tag = tagbase * k  + tag0; cnt_tag++;
            V[z].subtag = 0;
            /************************ update PQ when tag changed********************/
            //if (V[z].inQ) {PQ.push(DATA(z, V[z].tag, V[z].subtag));}
        }

        //reset w after relabel
        z = V[x].next;
        w = V[z].tag - V[x].tag; 
    }
    
    // in case x is the tail, scope the tag.
    if (w > INIT_TAG_GAP) w = INIT_TAG_GAP; 

    V[y].tag = (V[x].tag + w / 2); cnt_tag++;
    V[y].subtag = 0;
END:
    /************************ update PQ when tag changed********************/
    //if (V[y].inQ) {PQ.push(DATA(y, V[y].tag, V[y].subtag));} 

    ListInsert(x, y);
    return 0;
#endif
}

int SeqCM::CoreMaint::MultiOrderInsert(node_t x, vector<node_t> &y) {
#ifndef WITH_SUBTAG
    const size_t size = y.size();
    if (0 == size) return 0 ;
    else if (1 == size) {
        OrderInsert(x, y[0]);
        return 1;
    }

    unsigned int j = 1;
    node_t z = V[x].next;
    tag_t tag0 = V[x].tag;
    tag_t w = V[z].tag - tag0;
    
    if (unlikely(w <= size)) {
        //find j: 1 - j-1
        while (w <= j * j * size) { // could be geometric increasing.
            z = V[z].next;
            w = V[z].tag - V[x].tag;
            j++;
        }

        /*scope the tag range*/
        if (w > INIT_TAG_GAP * size * j) w = INIT_TAG_GAP * size * j;

        //relabel
        tag_t tagbase = w/j;
        for (size_t k = 1, z = V[x].next; k < j; k++, z=V[z].next) {
            V[z].tag = tagbase * k + tag0; cnt_tag++;
            // push into PQ again when tage changed. 
            //if (V[z].inQ) {PQ.push(DATA(z, V[z].tag));}
        }
        // reset label 
        z = V[x].next;
        w = V[z].tag - V[x].tag; //reset w
    }

    /*scope the tag range*/
    if (w > INIT_TAG_GAP * size) w = INIT_TAG_GAP * size;

    //update label
    w = w / (size + 1); 
    for (size_t i = 0; i < size; i++) {
        V[y[i]].tag = (V[x].tag + w * (i+1)); cnt_tag++;
        // push into PQ again when tag is changed. 
        //if (V[y[i]].inQ) {PQ.push(DATA(y[i], V[y[i]].tag));}
    }
    
    MultiListInsert(x, y);

    return 0;

#else // with the SUBTAG in MultiOrderInsert
    const size_t size = y.size();
    if (0 == size) return 0 ;
    else if (1 == size) {
        OrderInsert(x, y[0]);
        return 1;
    }

    unsigned int j = 1;
    node_t z = V[x].next;
    tag_t tag0 = V[x].tag;
    tag_t w = V[z].tag - tag0;
    
    if (unlikely(w <= size)) {
        const subtag_t subtag0 = V[x].subtag;
        subtag_t subw;
        if (w == 1) subw = MAX_SUBTAG - subtag0;
        else subw = V[z].subtag - subtag0; // w == 0
        if (subw > size) { // use the subtag
            w = w / (size+1);
            for (size_t i = 0; i < size; i++) {
                V[y[i]].tag = V[x].tag;
                V[y[i]].subtag = subtag0 + w * (i+1); cnt_tag++;
            }

            for (size_t i = 0; i < size; i++) {
                //if (V[y[i]].inQ) {PQ.push(DATA(y[i], V[y[i]].tag, V[y[i]].subtag));}
            }
            
            MultiListInsert(x, y);
            return 0;

        } 
        
        //find j: 1 - j-1
        while (w <= j * j * size) { // could be geometric increasing.
        //while(w <= j * size) {
            z = V[z].next;
            w = V[z].tag - V[x].tag;
            j++;
        }

        /*scope the tag range*/
        if (w > INIT_TAG_GAP * size * j) w = INIT_TAG_GAP * size * j;

        //relabel
        tag_t tagbase = w/j;
        for (size_t k = 1, z = V[x].next; k < j; k++, z=V[z].next) {
            V[z].tag = tagbase * k + tag0; cnt_tag++;
            V[z].subtag = 0;
            // push into PQ again when tage changed. 
            //if (V[z].inQ) {PQ.push(DATA(z, V[z].tag, V[z].subtag));}
        }
        // reset label 
        z = V[x].next;
        w = V[z].tag - V[x].tag; //reset w
    }

    /*scope the tag range*/
    if (w > INIT_TAG_GAP * size) w = INIT_TAG_GAP * size;

    //update label
    w = w / (size + 1); 
    for (size_t i = 0; i < size; i++) {
        V[y[i]].tag = (V[x].tag + w * (i+1)); cnt_tag++;
        V[y[i]].subtag = 0;
        // push into PQ again when tag is changed. 
        //if (V[y[i]].inQ) {PQ.push(DATA(y[i], V[y[i]].tag, V[y[i]].subtag));}
    }
    
    MultiListInsert(x, y);

    return 0;
#endif
}

int SeqCM::CoreMaint::OrderDelete(node_t x) {
    ListDelete(x);
    return 0;
}


int SeqCM::CoreMaint::EdgeInsert(node_t u, edge_t v) {
#ifdef DEBUG
    //deb("**************insert (%d, %d)************\n", u, v);
#endif

    
    // assuming order u < v
    if (!Order(u, v)) std::swap(u, v);
    deb("ADDING edge (%d, %d)\n", u, v);
    const core_t K = core[u];

    /*add edges*/
    graph[u].push_back(v); 
    graph[v].push_back(u);
    V[u].degout++;
    add_left_right_edge(u, v);
    if (V[u].degout <= K ) {
        if (core[u] <= core[v]) V[u].mcd++;
        if (core[v] <= core[u]) V[v].mcd++;
        return 1; // update mcd and return
    }

    PQ.clear(); Vblack.clear(); Vcolor.clear();
    PQ.push(u); V[u].inQ = true;

    /******* traverse with topological order***********/
    while (!PQ.empty()) {
        node_t w = PQ.top(); PQ.pop();//w has the smallest order
        if (unlikely(false == V[w].inQ) ) { continue; } //not in PQ, this is update the value of PQ.
        else { V[w].inQ = false; }
        
        if (V[w].degin + V[w].degout > K) { // forward
            Forward(w);
        } else if (V[w].degin > 0){ // backward (w must has degin > 0, or w can't be in PQ)
            Backward(w);
        } else {
            // Added to PQ but then removed by Backward
            for(int idx : adj[w].tmp_Vp_korder_less)
                adj[w].k_equal_korder_less_to_tmp[idx] = -1;
            adj[w].tmp_Vp_korder_less.clear();
            V[w].color = WHITE;
        }

    }

    /**ending phase**/

    /*update core number and mcd for BLACK nodes*/
    for (const node_t w: Vcolor){
        if (likely(V[w].color == BLACK)) { // in V*
            OrderDelete(w);// remove w from the order list O_k
            core[w]++;
            Vblack.push_back(w);
            for(auto k_eq : {&adj[w].k_equal_korder_less, &adj[w].k_equal_korder_more}) {
                for(int i = 0; i < k_eq->size();) {
                    const node_t x = (*k_eq)[i].v;
                    if (V[x].color == BLACK)
                        ++i;
                    else {
                        erase_edge(w, *k_eq, i);
                        add_left_right_edge(x, w);
                    }
                }
            }
            for(int i = 0; i < adj[w].k_more.size();) {
                const node_t x = adj[w].k_more[i].v;
                if (core[x] == core[w]) {
                    erase_edge(w, adj[w].k_more, i);
                    add_left_right_edge(w, x);
                } else
                    ++i;
            }
        } else { /* in V+ not in V* */ }
        for(int idx : adj[w].tmp_Vp_korder_less)
            adj[w].k_equal_korder_less_to_tmp[idx] = -1;
        adj[w].tmp_Vp_korder_less.clear();
    }

    // insert Vb2 (ordered black nodes) to the head of O_k+1
    MultiOrderInsert(H[K+1], Vblack);
    
    
    /*update the mcd*/
    if (Vblack.empty()) {
        if (core[u] <= core[v]) V[u].mcd++;
        if (core[v] <= core[u]) V[v].mcd++;
    } else {
        for (const node_t u: Vblack) {
            deg_t mcd = 0;
            for (const node_t v: graph[u]) {
                if (core[u] <= core[v]) mcd++;
                //u.core +1 for v. u->v
                if (BLACK != V[v].color && core[u] == core[v]) {
                    V[v].mcd++;
                }
            }
            V[u].mcd = mcd;
        }
    }

    // TODO: Remove
    for(const node_t w: Vcolor)
        verify_adj(w, true);

    //reset all color
    for(const node_t u: Vcolor) {
        V[u].color = WHITE;
        V[u].degin = 0;
        ASSERT(adj[u].tmp_Vp_korder_less.empty());
    }


    return 0;
}

void SeqCM::CoreMaint::Forward(node_t w){
    V[w].color = BLACK; cnt_Vs++; cnt_Vp++;// core number can be update 
    Vcolor.push_back(w); // all colored nodes
    for (auto& [w2, idx]: adj[w].k_equal_korder_more) { // w -> w2
        cnt_S++;
        ASSERT(SameCoreOrder(w, w2));
        V[w2].degin++;  // w is black
        verify_adj(w); verify_adj(w2);
        adj[w2].tmp_Vp_korder_less.push_back(idx);
        adj[w2].k_equal_korder_less_to_tmp[idx] = adj[w2].tmp_Vp_korder_less.size() - 1;
        deb("Forward: Adding edge (%d, %d)\n", w, w2);
        if (!V[w2].inQ) { // w2 is not in PQ
            PQ.push(w2); V[w2].inQ = true; 
        }
        V[w2].color = DARK;
    }
}

void SeqCM::CoreMaint::Backward(node_t w) { //???
    V[w].color = GRAY; cnt_Vp++; // white -> gray. 
    Vcolor.push_back(w); // all colored nodes. 
    
#ifdef WITH_V_BLACK2GRAY
    Vblack2gray.clear();
#else 
    node_t p = w;
#endif

    DoPre(w); //first time do black pre only;
    //V[w].degout += V[w].degin; V[w].degin = 0;

    V[w].color = WHITE;

    while (!R.empty()) { // Q has nodes from BLACK to GRAY 
        node_t u = R.top(); R.pop(); V[u].inR = false; 
        //V[u].degout += V[u].degin; V[u].degin = 0;
        V[u].color = GRAY;
        cnt_Vs--; // black -> gray
        

        DoAdj(u);

        V[u].color = WHITE;

        OrderDelete(u); 
#ifdef WITH_V_BLACK2GRAY
        Vblack2gray.push_back(u); 
#else
        OrderInsert(p, u); p = u;
#endif
    }

#ifdef WITH_V_BLACK2GRAY
    MultiOrderInsert(w, Vblack2gray);   
#endif      
}

void SeqCM::CoreMaint::inner_erase_edge(node_t u, vector<SeqCM::Edge> &es, int idx) {
    auto &a = adj[u];
    Edge &e = es[idx];
    if (idx != es.size() - 1) {
        Edge &back = es.back();
        if (&es == &a.k_less) adj[back.v].k_more[back.edge_idx_in_v].edge_idx_in_v = idx;
        else if (&es == &a.k_more) adj[back.v].k_less[back.edge_idx_in_v].edge_idx_in_v = idx;
        else if (&es == &a.k_equal_korder_less) {
            adj[back.v].k_equal_korder_more[back.edge_idx_in_v].edge_idx_in_v = idx;
            int prev_idx = a.k_equal_korder_less_to_tmp.back();
            if (prev_idx != -1) {
                a.tmp_Vp_korder_less[prev_idx] = idx;
                a.k_equal_korder_less_to_tmp[idx] = prev_idx;
            }
        } else if (&es == &a.k_equal_korder_more) adj[back.v].k_equal_korder_less[back.edge_idx_in_v].edge_idx_in_v = idx;
        else assert(false);
        es[idx] = es.back();
    }
    if (&es == &a.k_equal_korder_less) a.k_equal_korder_less_to_tmp.pop_back();
    es.pop_back();
}

void SeqCM::CoreMaint::erase_edge(node_t u, vector<Edge> &es, int idx) {
    auto &a = adj[u];
    Edge &e = es[idx];
    if (&es == &a.k_less) inner_erase_edge(e.v, adj[e.v].k_more, e.edge_idx_in_v);
    else if(&es == &a.k_more) inner_erase_edge(e.v, adj[e.v].k_less, e.edge_idx_in_v);
    else if(&es == &a.k_equal_korder_less) inner_erase_edge(e.v, adj[e.v].k_equal_korder_more, e.edge_idx_in_v);
    else if(&es == &a.k_equal_korder_more) inner_erase_edge(e.v, adj[e.v].k_equal_korder_less, e.edge_idx_in_v);
    else assert(false);
    inner_erase_edge(u, es, idx);
}

void SeqCM::CoreMaint::DoPre(node_t u) {
    for (int idx : adj[u].tmp_Vp_korder_less) {
        node_t v = adj[u].k_equal_korder_less[idx].v;
        if (WHITE == V[v].color) {  continue; }
        assert(V[v].color == BLACK);
        verify_adj(u); verify_adj(v);
        V[v].degout--;
        ASSERT(V[v].degout >= 0);
        V[u].degin--; V[u].degout++;
        assert(V[u].degin >= 0);
        deb("DoPre: Removing edge (%d, %d) and moving forward\n", u, v);
        erase_edge(u, adj[u].k_equal_korder_less, idx);
        add_left_right_edge(u, v);


        if (!V[v].inR && BLACK == V[v].color  && V[v].degin + V[v].degout <= core[v]) {
            deb("Adding v %d to R\n", v);
            R.push(v); V[v].inR = true;
        }
    }   
}


/*void SeqCM::CoreMaint::verify_adj(node_t u, bool deep) {
    // TODO: REMOVE, this is inneficcient
    vector<node_t> new_adj;
    auto &a = adj[u];
    assert(V[u].degout == a.k_equal_korder_more.size() + a.k_more.size());
    assert(adj[u].k_equal_korder_less_to_tmp.size() == a.k_equal_korder_less.size());
    for (int i = 0; i < a.k_less.size(); i++) {
        auto &[v, idx] = a.k_less[i];
        assert(!deep || core[v] < core[u]);
        assert(adj[v].k_more[idx].v == u && adj[v].k_more[idx].edge_idx_in_v == i);
        new_adj.push_back(v);
    }
    for (int i = 0; i < a.k_equal_korder_less.size(); i++) {
        auto &[v, idx] = a.k_equal_korder_less[i];
        assert(!deep || SameCoreOrder(v, u));
        assert(adj[v].k_equal_korder_more[idx].v == u && adj[v].k_equal_korder_more[idx].edge_idx_in_v == i);
        new_adj.push_back(v);
    }
    for (int i = 0; i < a.k_equal_korder_more.size(); i++) {
        auto &[v, idx] = a.k_equal_korder_more[i];
        assert(!deep || SameCoreOrder(u, v));
        assert(adj[v].k_equal_korder_less[idx].v == u && adj[v].k_equal_korder_less[idx].edge_idx_in_v == i);
        new_adj.push_back(v);
    }
    for (int i = 0; i < a.k_more.size(); i++) {
        auto &[v, idx] = a.k_more[i];
        assert(!deep || core[u] < core[v]);
        assert(adj[v].k_less[idx].v == u && adj[v].k_less[idx].edge_idx_in_v == i);
        new_adj.push_back(v);
    }
    std::sort(new_adj.begin(), new_adj.end());
    for(int i = 0; i < int(new_adj.size()) - 1; i++)
        assert(new_adj[i] != new_adj[i+1]);
    std::sort(graph[u].begin(), graph[u].end());
    ASSERT(new_adj.size() == graph[u].size());
    ASSERT(!deep || a.tmp_Vp_korder_less.empty());
    for(int v: graph[u])
        ASSERT(std::binary_search(new_adj.begin(), new_adj.end(), v));
}*/


void SeqCM::CoreMaint::DoAdj(node_t u) {
    for (auto &[v, _] : adj[u].k_equal_korder_more) {
        //ASSERT(SameCoreOrder(u, v));
        
        if (WHITE == V[v].color) continue;

        V[v].degin--;
        ASSERT(V[v].degin >= 0);
        deb("DoAdj: Removing edge (%d, %d)\n", u, v);

        if (!V[v].inR && BLACK == V[v].color && V[v].degin + V[v].degout <= core[v]) {
            deb("Adding v %d to R\n", v);
            //V[v].color = GRAY; 
            R.push(v); V[v].inR = true;
        }
    }
    DoPre(u);
}


int SeqCM::CoreMaint::EdgeRemove(node_t u, edge_t v) {
#ifdef DEBUG
    deb("**************remove (%d, %d)************\n", u, v);
#endif

    const core_t K = std::min(core[u], core[v]);
    if (!Order(u, v)) std::swap(u, v);

    //remove the edge
#if 1  // this erase is much faster. why? May because the STL has high efficiency.
    graph[u].erase(std::find(graph[u].begin(), graph[u].end(), v));
    graph[v].erase(std::find(graph[v].begin(), graph[v].end(), u));
#else 
    //vector<edge_t> &uedge = graph[u];
    size_t size = graph[u].size();
    for (size_t i = 0; i < size; i++) {
        if (unlikely(graph[u][i] == v)){
            std::swap(graph[u][i], graph[u][size - 1]); 
            graph[u].pop_back();
            break;
        }
    }
    //vector<edge_t> &vedge = graph[v];
    size = graph[v].size();
    for (size_t i = 0; i < size; i++) {
        if (unlikely(graph[v][i] == u)) {
            std::swap(graph[v][i], graph[v][size - 1]); 
            graph[v].pop_back();
            break;
        }
    } 
#endif 

    //update mcd value
    if (core[u] <= core[v]) {
        V[u].mcd--;
        if (core[u] > V[u].mcd) {
            V[u].color = BLACK; Vblack.push_back(u); cnt_Vs++;
            R.push(u); 
        }
    } 
    if (core[v] <= core[u]) {
        V[v].mcd--;
        if (core[v] > V[v].mcd) {
            V[v].color = BLACK; Vblack.push_back(v); cnt_Vs++;
            R.push(v); 
        }
    }
    
    if (Order(u, v)) V[u].degout--;

    //V* is empty. no core number affected.
    if (R.empty()) { return 0; }

    //find V* 
    while(!R.empty()) {
        node_t w = R.top(); R.pop();
        for (const node_t w2: graph[w]) {
            if (K != core[w2]) continue; // search within the the range
            if (BLACK == V[w2].color) continue; // search jump over the black
            V[w2].mcd--;
            if (core[w2] > V[w2].mcd) {
                ASSERT(WHITE == V[w2].color);
                V[w2].color = BLACK; Vblack.push_back(w2); cnt_Vs++;
                R.push(w2);
            }
        }    
    }

    /***ending phase***/ 
    //reset degout from the paper : fast order based ...
    //update core number in V*
    for (const edge_t w: Vblack) { core[w]--; }

    for (const node_t w: Vblack) {
        V[w].degout = 0;
        for (const node_t w2: graph[w]) {
            cnt_S++;
            if (K == core[w2] && TagOrder(w2, w)) 
                V[w2].degout--;
            if (core[w2] >= K || BLACK == V[w2].color) 
                V[w].degout++;
        }
        V[w].color = WHITE;
        OrderDelete(w); 
        //OrderInsert(V[T[K-1]].pre, w);
    }

    MultiOrderInsert(V[T[K-1]].pre, Vblack);
    
    //UPDATE mcd;
    for (const node_t w: Vblack) {
        deg_t mcd = 0;
        for (const node_t w2: graph[w]) {
            if (core[w]<=core[w2]) mcd++;
        }
        V[w].mcd = mcd;
    } 

   //reset Vblack 
    Vblack.clear();

    return 0;
}

/*x, y: inserted edge
*id: the id of inserted edge
* check each time to find bug*/
int SeqCM::CoreMaint::Check(node_t x, node_t y, int id, vector<core_t> &tmp_core, vector<node_t> &order_v) {

    deb("*********** Our Check **************\n");

        //check core number
#ifdef DEBUG
    for(int i = 0; i < n; i++) {
        if(tmp_core[i] != core[i]){
            deb("wrong! *** %d: core is %d but %d after I/R %d edges \n", i, core[i], tmp_core[i], id);
        }
    }
#else
    ASSERT_INFO(tmp_core == core, "wrong result after insert");
#endif

#ifndef DEBUG
    return 0; // only check the core number for release version. 
#endif
    int error = 0;
    //check reverse order list  
    size_t total_ver_num = 0;
    for(size_t k = 0; k < H.size(); k++){
        vector<node_t> order, rorder;
        for(node_t v = V[H[k]].next; v != T[k]; v = V[v].next) {
            order.push_back(v);
            total_ver_num++;
        }

        for(node_t v = V[T[k]].pre; v != H[k]; v = V[v].pre) {
            rorder.push_back(v);
        }
        std::reverse(rorder.begin(), rorder.end());

        ASSERT(order == rorder);
    }
    if (total_ver_num != n) {
#ifdef DEBUG
        deb("wrong! *** total number in list: %ld but %ld", total_ver_num, n);
#else
        ASSERT(false);
#endif
    }


    //check order list core number 
    for(int k = 0; k < (int)H.size(); k++){
        for(node_t v = V[H[k]].next; v != T[k]; v = V[v].next) {
            if (k != core[v]) {
#ifdef DEBUG
                deb("wrong! *** list check %d has core number %d but should be %d\n", v, k, core[v]);    
#else
                ASSERT(false);
#endif
            }
        }
    }


    //check tag
    // vector<int> order_v_index; int order = 0;
    // for (const node_t v: order_v) { // order_v of order
    //     order_v_index.push_back(order++);
    // } 

    for(size_t k = 0; k < H.size(); k++){
        for(node_t v = V[H[k]].next; v != T[k]; v = V[v].next) {
            node_t next = V[v].next;

            // if (next != T[k] && order_v_index[v] >= order_v_index[next]) {
            //     ASSERT(false);
            // }

            if (V[v].tag >= V[next].tag) {
#ifdef DEBUG
            deb("k=%d, %d.tag: %llu, next %d.tag: %llu\n",
                k, v, V[v].tag, next, V[next].tag);
#else
            ASSERT(false);
#endif
            }

        }

        if (V[H[k]].tag != 0) {
#ifdef DEBUG
            node_t next = V[H[k]].next;
            deb("%d head.tag: %llu, next  %d.tag: %llu\n",
                k, V[H[k]].tag, next, V[next].tag);
#else
            ASSERT(false);
#endif
        }
        if(V[T[k]].tag != MAX_TAG) {
            ASSERT(false);
        }
        
    }

    //check degout degin core mcd
    for (size_t u = 0; u < n; u++){
        // degout <= core number
        if (V[u].degout > core[u]) {
#ifdef DEBUG
            deb("degout [%d (%d, %d)]:  %d.degout = %d, core[%d]=%d\n",
                id, x, y, u, V[u].degout, u, core[u]);
#else
            ASSERT(false);
#endif
        }

        // check the degout and degin
        if (0 != V[u].degin){
#ifdef DEBUG
            deb("degin [%d (%d, %d)]:  %d.degin = %d, should be %d\n",
                id, x, y, u, V[u].degin, 0);
#else
            ASSERT(false);
#endif
        }
        deg_t degout = 0, mcd = 0;
        for (const node_t v: graph[u]){
            if (Order(u, v)) degout++;
            if (core[u] <= core[v]) mcd++;
        }
        if (degout != V[u].degout) {
#ifdef DEBUG
            deb("EDGE [%d (%d, %d)]:  %d.degout = %d, should be %d\n",
                id, x, y, u, V[u].degout, degout);
#else 
            ASSERT(false);
#endif
        }
        if (mcd != V[u].mcd) { //check mcd
#ifdef DEBUG
            deb("EDGE [%d (%d, %d)]:  %d.mcd = %d, should be %d\n",
                id, x, y, u, V[u].mcd, mcd);
#else 
            ASSERT(false);
#endif           
        }
    }

    return error;

}

}