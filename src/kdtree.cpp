#include <kdtree.h>

#include <algorithm>
#include <vector>
#include <cmath>
#include <tuple>
#include <unordered_map>
#include <stack>
#include <queue>
#include <cstring>
#include <cassert>
#include <cstdlib>
#include <iostream>
// python : https://leileiluoluo.com/posts/kdtree-algorithm-and-implementation.html

// ref: https://www.jianshu.com/p/80e41da2a397
// https://github.com/WiseDoge/libkdtree/blob/master/src/kdtree.cpp
// https://leileiluoluo.com/posts/kdtree-algorithm-and-implementation.html
// https://blog.csdn.net/image_fzx/article/details/80624968

// Example :
//         int x = Malloc(int, 10)
//         int y = (int*)malloc(10 * sizeof(int))
#define Malloc(type, n) (type * )malloc((n)*sizeof(type))

// // 释放一颗二叉树内存的非递归算法
// 这一段代码是有背诵意义的

void free_tree_memory(tree_node *root) {
    std::stack<tree_node*> node_stack;
    tree_node *p;
    node_stack.push(root);
    while (!node_stack.empty()) {
        p = node_stack.top();
        node_stack.top();
        if (p->left) 
            node_stack.push(p->left);
        if (p->right)
            node_stack.push(p->right);
        free(p);
    }
}
template<typename T>
class KDTree {
public:
    KDTree() {std::cout << "Default Constructor" << std::endl;}
    KDTree(const T *datas, size_t rows, size_t cols);
    ~KDTree();

    tree_node *GetRoot() {return root; }
    std::vector<std::tuple<size_t, T>> FindKNearests(const T *coor, size_t k);
    std::tuple<size_t, T> FindNearest(const T *coor, size_t k) {
        return FindKNearests(coor, k)[0];
    } 
    void CFindNearests(const T *coor, size_t k, size_t *args, T *dists);
private:
    // The sample with the largest distance from point `coor` is
    // always at the top of the heap
    // 这里是大顶堆
    struct neighbor_heap_cmp {
        bool operator() (const std::tuple<size_t, T> &i,
                         const std::tuple<size_t, T> &j) {
            return std::get<1>(i) < std::get<1>(j);                  
        }
    };
    typedef std::tuple<size_t, T> neighbor;  
    typedef std::priority_queue<neighbor, std::vector<neighbor>, neighbor_heap_cmp> neighbor_heap; 
    // 搜索K-近邻时的堆(大顶堆), 堆顶始终是K-近邻中样本点最远的点 
    neighbor_heap k_neighbor_heap_; 
    // 求距离时候的p, dist(x, y) = pow((x^p + y^p), 1/p)
    T p;
    // 析构时候是否释放树的内存
    bool free_tree_;
    // 树根结点
    tree_node *root;
    // 训练集
    const T *datas;
    // 训练集的样本数
    size_t n_samples;
    // 每个样本的维度
    size_t n_features;
    // 寻找中位数时用到的缓存池
    std::tuple<size_t, T> *get_mid_buf_;
    // 搜索k近邻时候的缓存池, 如果已经搜索过点i, 那么令 visited_buf[i] = True
    bool *visited_buf_;
    // 初始化缓存
    void InitBuffer();

    // 建树
    tree_node *BuildTree(const std::vector<size_t> &points);

    // 求一组数字的中位数
    std::tuple<size_t, T> MidElement(const std::vector<size_t> &points, size_t dim);
    // 入堆
    void HeapStackPush(std::stack<tree_node*> &paths, tree_node *node, const T *coor, size_t k);
    // 获取训练集中第sample个样本点第dim的值
    T GetDimVal(size_t sample, size_t dim) {
        return datas[sample * n_features + dim];
    }
    // 求点coor距离训练集第i个点的距离
    T GetDist(size_t i, const T* coor);
    //  寻找切分点
    size_t FindSplitDim(const std::vector<size_t> &points);
    
};
// 寻找最近点的算法
// template<typename T>
// std::vector<std::tuple<size_t, T>> KDTree
// 深度优先使用堆栈
// coor 应该是查找点P的坐标, 在kdtree中查找距离点P最近的K个点s, 设L为一个有k个空位的列表
// 用于保存已经搜索到的最近点   
// KNN算法
// 1. 根据p的坐标值和每个节点的切分向下搜索, 也就是说如果树的节点是按照x_{r} = a进行切分
// 并且p的r坐标小于a, 则向左枝进行搜索, 反之则向右枝进行搜索
// 2. 当达到一个底部节点的时候, 将其标记为已经访问过了。 如果L里面不足k个点,  
template <typename T> 
std::vector<std::tuple<size_t, T>> KDTree<T>::FindKNearests(const T* coor, size_t k) {
    // 好好学一下下面动态分配内存的方法
    std::memset(visited_buf_, 0, sizeof(bool) *n_samples);
    std::stack<tree_node*> paths;
    tree_node *p = root; 
    // 1. 从上往下寻找路径一直到找到叶子节点
    while (p) {
        HeapStackPush(paths, p, coor, k);
        p = coor[p->split] <= GetDimVal(p->id, p->split) ? p->left : p->right;
    }
    // 2. 从下往上递归遍历保留距离最近的k个点, 一直到
    while (!paths.empty()) {
        p = paths.top();
        paths.pop();
        // 叶子节点不需要干什么了, 然后往上面一层, 叶子节点已经被访问过  
        if (!p->left && !p->right) {
            continue;
        }
        // 下面只是针对于  
        if (k_neighbor_heap_.size() < k) {
            if (p->left) {
                HeapStackPush(paths, p->left, coor, k); 
            }
            if (p->right) {
                HeapStackPush(paths, p->right, coor, k);
            }
        } else {
            T node_split_val = GetDimVal(p->id, p->split); // p是path路径上面的点， 在递归的过程中得出
            T coor_split_val = coor[p->split]; // 测试点在p这个维度的坐标值
            T heap_top_val = std::get<1>(k_neighbor_heap_.top());
            if (coor_split_val > node_split_val) { // coor在右边子树上, 
                if (p->right) 
                    HeapStackPush(paths, p->right, coor, k);
                if ((coor_split_val - node_split_val) < heap_top_val) {
                    HeapStackPush(paths, p->left, coor, k);
                }
            } else {
               if (p->left)
                    HeapStackPush(paths, p->left, coor, k);
               if ((node_split_val - coor_split_val) < heap_top_val && p->right)
                    HeapStackPush(paths, p->right, coor, k);
            }
        }

    }
    // 3. 找到最近的k个点后, 将最近的k个点全部放进一个vector中并且返回
    std::vector<std::tuple<size_t, T> > res;
    while (!k_neighbor_heap_.empty()) {
        res.emplace_back(k_neighbor_heap_.top());
        k_neighbor_heap_.pop();
    }
    return res;
}

// 2020.01.01. 完成kd Tree的构造函数, 
// 树的构建算法, 循序渐进地选取数据点的各个维度来作为切分维度, 取数据点在该维度的中间数值作为
// 切分超平面, 将中间数值左侧的数据点挂在其左子树, 将中间数值右侧额数据点挂在其右边子树,
// 递归的处理子树,一直到其所有数据点挂载完毕
template<typename T>
KDTree<T>::KDTree(const T *datas, size_t rows, size_t cols):
                 datas(datas), n_samples(rows), n_features(cols) {
    std::vector<size_t> points;
    for (size_t i = 0; i < n_samples; ++i) {
        points.emplace_back(i);
    }
    InitBuffer();
    root = BuildTree(points);
    std::cout << GetDimVal(root->id, 0) << std::endl;
    std::cout << GetDimVal(root->id, 1) << std::endl;    
}
template<typename T>
void KDTree<T>::InitBuffer() {
    get_mid_buf_ = new std::tuple<size_t, T>[n_samples];
    visited_buf_ = new bool[n_samples];
}
// 递归地去构建树
template<typename T> 
tree_node* KDTree<T>::BuildTree(const std::vector<size_t> &points) {
    // 1. 找到要分割的维度, 应该在0到n_features之间   
    size_t dim = FindSplitDim(points);
    std::cout << "Cutoff Dim" << dim << std::endl;
    // 2.  找到dim维度上中间的点 
    std::tuple<size_t, T> t = MidElement(points, dim);
    size_t arg_mid_val = std::get<0>(t);
    T mid_val = std::get<1>(t);
    // 3. 建立根节点
    tree_node *node = Malloc(tree_node, 1);
    node->left = nullptr;
    node->right = nullptr;
    node->id = arg_mid_val;
    node->split = dim;
    std::vector<size_t> left, right;
    for (auto &i : points) {
        if (i == arg_mid_val) 
            continue;
        if (GetDimVal(i, dim) <= mid_val)
            left.emplace_back(i);
        else 
            right.emplace_back(i);
    }
    if (!left.empty()) {
        node->left = BuildTree(left);
    }
    if (!right.empty()) {
        node->right = BuildTree(right);
    }
    return node;
}
// 寻找中间节点
// https://blog.csdn.net/smf0504/article/details/51426750
template<typename T>
std::tuple<size_t, T> KDTree<T>::MidElement(const std::vector<size_t>& points, size_t dim) {
    size_t len = points.size(); // 数据点的个数
    for (size_t i = 0; i < len; ++i) {
        get_mid_buf_[i] = std::make_tuple(points[i], GetDimVal(points[i], dim));
    }
    std::nth_element(get_mid_buf_,
                    get_mid_buf_ + len / 2,
                    get_mid_buf_ + len,
                    [](const std::tuple<size_t, T> &i, const std::tuple<size_t, T> &j){
                        return std::get<1>(i) < std::get<1>(j);
                    });
    return get_mid_buf_[len / 2];  
}

// 下面这个函数有需要的话我觉得是需要重写的
// 寻找切分维度和切分点
// 一般的, 我们可以选取dim = floor % n_features， 也就是说当前树的层数对特征数取余数 
// 我们在这里使用dim = argmax(nmax - nmin)
// 也就是说选取当前节点集合中极差最大的维度   
// dim说到底就是余数啊     
template<typename T>
size_t KDTree<T>::FindSplitDim(const std::vector<size_t> &points) {
    // 只有一个数据点
    if (points.size() == 1)
        return 0;
    // 初始化维度
    size_t cur_best_dim = 0;
    T cur_largest_spread = -1;
    T cur_min_val;
    T cur_max_val;
    for (size_t dim = 0; dim < n_features; ++dim) {
        cur_min_val = GetDimVal(points[0], dim);
        cur_max_val = GetDimVal(points[1], dim);
        for (const auto &id : points) {
            if (GetDimVal(id, dim) > cur_max_val) 
                cur_max_val = GetDimVal(id, dim);
            else if (GetDimVal(id, dim) < cur_min_val) 
                cur_min_val = GetDimVal(id, dim);
        }
        if (cur_max_val - cur_min_val > cur_largest_spread) {
            cur_largest_spread = cur_max_val - cur_min_val;
            cur_best_dim = dim;
        }
    }
    return cur_best_dim;   
}
// 析构函数
template<typename T>
KDTree<T>::~KDTree() {
    delete[] get_mid_buf_;
    delete[] visited_buf_;
    free_tree_memory(root);
}
// 辅助函数, 用来放二叉树遍历过程中的中间变量
// 我们在找最近的k个点的时候使用了递归的方法
// HeapStacksPush
// 用来维持

template<typename T>
void KDTree<T>::HeapStackPush(std::stack<tree_node*> &paths, tree_node *node, 
                            const T* coor, size_t k) {
    paths.emplace(node);
    size_t id = node->id; 
    if (visited_buf_[id])
        return;
    visited_buf_[id] = true;
    T dist = GetDist(id, coor);
    std::tuple<size_t, T> t(id, dist);// 第一个是数据的id号码, 第二个是距离
    if (k_neighbor_heap_.size() < k) {
        k_neighbor_heap_.push(t);
    }  
    // 大顶堆 
    else if (std::get<1>(t) < std::get<1>(k_neighbor_heap_.top())) {
        k_neighbor_heap_.pop();
        k_neighbor_heap_.push(t);
    }
}
template<typename T>
T KDTree<T>::GetDist(size_t i, const T *coor) {
    T dist = static_cast<T>(0.0);
    size_t idx = i * n_features;    
    for  (int t = 0; t < n_features; ++t) {
        dist += std::pow(datas[idx + t] - coor[t], 2);    
    }
    return static_cast<T>(pow(dist, 0.5));   
}
template<typename T>
void KDTree<T>::CFindNearests(const T *coor, size_t k, size_t *args, T *dists) {
    std::vector<std::tuple<size_t, T>> k_nearest = FindKNearests(coor, k);
    for (size_t i = 0; i < k; i++) {
        args[i] = std::get<0>(k_nearest[i]);
        dists[i] = std::get<1>(k_nearest[i]);
    }
    return; 
}
template<typename T>
tree_model<T>* build_kdtree(const T* datas, size_t rows, size_t cols)
              { 
    KDTree<T> tree(datas, rows, cols);
    tree_model<T> *model = Malloc(tree_model<T>, 1);
    model->datas = datas;
    model->n_features = cols;
    model->n_samples = rows;
    model->root = tree.GetRoot();
    // TODO: add p for calculating the distance     
    return model;   
}
// 构造kd树的任务完成
int main(int argc, char* argv[]) {
    tree_node* tn1 = new tree_node;
    // https://zhidao.baidu.com/question/599437673.html
    // tn1->left = NULL;
    // tn1->right = NULL;
    tn1->id = 1;
    std::cout << tn1->id << std::endl;
    tree_model<float>* tm1 = new tree_model<float>;
    tm1->root  = tn1;
    // 做一个Test Case 
    // 使用一些二维的数据点, 写成矩阵的形式   
    KDTree<float>* kd_tree = new KDTree<float>; 
    float datas[] = {2, 3, 5, 4, 9, 6, 4, 7, 8, 1, 7, 2};
    size_t rows = 6, cols = 2;
    KDTree<float>* kd_tree1 = new KDTree<float>(datas, rows, cols);
    // test algorithm 
    // 参考博客为 https://www.joinquant.com/view/community/detail/2843 
    // 一共七个数据点, 维度为2
    float data_set[] = {6.27, 5.50, 1.24, -2.86, 17.05, -12.79, -6.88, -5.40, -2.96, -0.50, 7.75, -22.68, 10.80, -5.03, -4.60, -10.55,
                        -4.96, 12.61, 1.75, 12.26, 15.31, -13.16, 7.83, 15.70, 14.3, -0.35};
    size_t row_data = 13, col_data = 2;
    KDTree<float>* kd_tree2 = new KDTree<float>(data_set, row_data, col_data);
    float p_piont[] = {-1, -5}; 
    // 最近的三个点
    // auto res = kd_tree2->FindKNearests(p_piont, 3);
    size_t res_index[3];
    float res_dist[3];
    kd_tree2->CFindNearests(p_piont, 3, res_index, res_dist);
    for (int i = 0; i < 3; i++) {
        std::cout << res_index[i] << std::endl;
    }    
    return 0;
}

