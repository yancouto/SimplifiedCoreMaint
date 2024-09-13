#pragma once
#include <vector>
#include <queue>
#include <math.h>       /* pow */
#include <climits>
#include <map>
using namespace std;

extern int cnt_PQ;

namespace yan {


//for graph
const size_t MAX_INCREASE_CORE = 1024*16; //max increased core number, should be large enough
const size_t VECTOR_CAPACITY = 1024*1024;
typedef int node_t; // the vertex of graph
const node_t NONE = -1; 
typedef int edge_t; // the edge of graph
typedef int core_t; // the core number of graph 
typedef int deg_t;  // degree
typedef int color_t;
typedef int label_t;
typedef int worker_t;
typedef unsigned int counter_t;
typedef vector<vector<edge_t>> graph_t; 

// in the BackWard function
/*#define WITH_V_BLACK2GRAY*/

/*#define DEBUG_ORDER_LIST */
#ifdef DEBUG_ORDER_LIST
typedef unsigned long long int tag_t; // 64bit label tag;
const size_t MAX_TAG = 0xffffffffffffffff;
const size_t INIT_TAG_GAP = 10; // 32 bit integer
#else 

/*one tag is easy to implemented. 
* So we can use 128 bit tag to replace the double level tags.
* The 128 bit tag can insert 64 item at least between */
#if 1

typedef unsigned long long int tag_t; // 64bit label tag;
typedef unsigned int subtag_t; // 32bit label subtag;
const size_t MAX_TAG = 0xffffffffffffffff;
const size_t INIT_TAG_GAP = 0xffffffff; // 32 bit unsigned integer
const size_t MAX_SUBTAG = 0xffffffff; // 32 bit unsigned integer
/*#define WITH_SUBTAG */ // this is defined in makefile

#else // !!!my machine doesn't support this __int128 well. 
typedef unsigned __int128 tag_t; //128bit label tag;
const size_t MAX_TAG = 0xffffffffffffffffffffffffffffffff;
const size_t INIT_TAG_GAP = 0xffffffffffffffff;
#endif 
//typedef unsigned int subtag_t;

#endif 

namespace SeqCM {
    enum{
        WHITE   = 0, /*initial*/
        DARK = 3, /*visited in PQ wating to be propagated*/
        GRAY    = 1, /*visited, not in V* but in V+*/
        BLACK   = 2, /*visited, in V* and in V+*/
    };

    class Node {
    public:
        //Computer Core number
        deg_t degout; // deg+
        deg_t degin;  // deg*
        deg_t mcd;    //for edge removing.
       
        //Order Maintenance
        node_t pre;  //double linked list
        node_t next; //double linked list
        tag_t tag;   // label tag for each item range is 0 - n^2
#ifdef WITH_SUBTAG
        subtag_t subtag; // range 0 to logn.
#endif

        color_t color; // black, white, gray.
        label_t inQ;   // node is in priority queue PQ
        label_t inR;    // node is in R can be defined with color
        //label_t outR;   // node is already backward.
        Node(){
            degout = degin = mcd = tag = 0; // init to 0 
#ifdef WITH_SUBTAG
            subtag =0;
#endif
            pre = next = NONE;
            color = WHITE;
            inQ = false;
            inR = false; 
        }
    };
}

struct CompareOrder {
    vector<SeqCM::Node> &V;
    bool operator()(node_t const& a, node_t const& b) 
    { 
        tag_t tag_a = V[a].tag;
        tag_t tag_b = V[b].tag;
        // return "true" if "a" is ordered  
        // before "b", for example:
#ifdef WITH_SUBTAG
        if (likely(tag_a != tag_b))
            return tag_a > tag_b;
        else 
            return V[a].tag2 > V[b].tag2; 
            
#else
        return tag_a > tag_b;
#endif
    } 
};

template<typename T>
std::vector<T> reserved_vec(size_t size) {
    std::vector<T> v;
    v.reserve(size);
    return std::move(v);
}

class PRIORITY_Q {
private:
    typedef std::priority_queue<node_t, std::vector<node_t>, CompareOrder> PQ;
    PQ pq_;
    std::vector<SeqCM::Node> &V;

public:
    PRIORITY_Q(size_t size, std::vector<SeqCM::Node> &V): pq_(CompareOrder{V}, std::move(reserved_vec<node_t>(size))), V(V) {}
    inline void push(node_t d) { pq_.push(d); cnt_PQ++;}
    inline node_t top() {return pq_.top();}
    inline void pop() {pq_.pop();}
    inline bool empty() {return pq_.empty(); }
    inline void clear() { while (!pq_.empty()){pq_.pop();} }
};
/**priority queue for core maint end *************/


#if 1
/**queue for core maint begin*********************/
// this queue can be vector
class QUEUE {
private:
    std::vector<node_t> container;
public:
    QUEUE(size_t size) {
        container.reserve(size);
    }
    QUEUE() {}
    inline void push(node_t v) {container.push_back(v);}
    inline node_t top() {return container.back();}
    inline void pop() {container.pop_back();}
    inline bool empty() {return container.empty(); }
    inline void clear() { container.clear();}
};
/***queue for core maint end*********************/
#endif
#if 0 // this implementation is more efficient
class QUEUE {
private:
    //std::vector<node_t> container;
    node_t *container = NULL;
    int ptr = 0;
public:
    QUEUE(size_t size) {
        //container = std::vector<node_t>(size);
        container = new node_t[size];
    }
    //~QUEUE(){delete[] container;}
    QUEUE() {}
    inline void push(node_t v) {container[ptr++] = v;}
    inline node_t top() {return container[ptr-1];}
    inline void pop() {ptr--;}
    inline bool empty() {return 0 == ptr; }
    inline void clear() { ptr = 0;}
};
#endif


namespace SeqCM{
    
    class Edge {
        public:
        node_t v;
        size_t edge_idx_in_v;
    };
    
    class PartitionedAdjacencyList {
        public:
        vector<Edge> k_less;
        vector<Edge> k_more;
        vector<Edge> k_equal_korder_less;
        vector<Edge> k_equal_korder_more;
        // This is exactly the nodes that count in degin
        vector<node_t> tmp_Vp_korder_less;
    };

    /*first version, with one level tags (label), only tag is used.
    * take O(logn) time for insert*/
    class CoreMaint {
        private:
        size_t n; // the number of vertices in graph.
        core_t max_core; // the max core number
        graph_t& graph;
        vector<core_t>& core;
        vector<PartitionedAdjacencyList> adj;
        vector<Node> V; // node information include head and tail
        // head-> x -> y -> tail (easy for insert and remove)
        
        /*maintaining the order   head(avoid empty list check)->node->node(tail)*/
        vector<core_t> H;  //each core list has a head to avoid empty list  
        vector<core_t> T;  //each core list has a tail;
        
        /*list operation is here*/
        inline void ListDelete(node_t x);
        inline void ListInsert(node_t x, node_t y);
        inline void MultiListInsert(node_t x, vector<node_t> &y);
        //inline void ListLink(vector<node_t> &y); //link all nodes in x.

        /*order operation*/
        inline bool Order(node_t x, node_t y); // return x < y
        inline bool SameCoreOrder(node_t x, node_t y);
        inline bool TagOrder(node_t x, node_t y) {return V[x].tag < V[y].tag;}
        int OrderInsert(node_t x, node_t y); // insert y after x
        int MultiOrderInsert(node_t x, vector<node_t> &y);
        inline int OrderDelete(node_t x);   // remove x
        void verify_adj(node_t u, bool deep=false);
        void add_left_right_edge(node_t u, node_t v);
        void erase_edge(node_t u, vector<Edge> &edges, int idx);
        void inner_erase_edge(node_t u, vector<Edge> &edges, int idx);


        QUEUE R; 
      
        PRIORITY_Q PQ;      // queue backward
        
        vector<node_t> Vblack;  // the order of black nodes (may include gray)
#ifdef WITH_V_BLACK2GRAY
        vector<node_t> Vblack2gray; // for ordered black to gray nodes  
#endif
        vector<node_t> Vcolor; // all colored vertices.
        
        public:
        /*the insert and remove algorithm*/
        CoreMaint(size_t n, graph_t &graph, vector<core_t> &core);
        ~CoreMaint(){}
        void Init(vector<int> &odv);
        void ComputeCore(graph_t &graph, vector<int> &core, vector<int> &order_v, bool with_init);
        
        // for edge insert
        inline void Forward(node_t w);
        inline void Backward(node_t w);
        inline void DoPre(node_t u);
        inline void DoAdj(node_t u);
        int EdgeInsert(node_t x, edge_t y); // insert to double-linked list
        
        inline void BeginRemove(node_t u, node_t v);
        inline void BeginRemoveContinue(node_t u, node_t v);
        inline void FindVs(const core_t K);
        inline void AfterRemove(const core_t K);
        int EdgeRemove(node_t x, edge_t y); //remove in double-linked list.
        
        // check the correctness of algorithm
        int Check(node_t, node_t, int id, vector<core_t> &tmp_core, vector<node_t> &order_v); 
    };

}


}