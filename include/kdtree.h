#ifndef KDTREE_H
#define KDTREE_H
#include <stdio.h>
// Change Log: 2020年定义完成kd树基本数据结构
// kd 树数据结构, 整理清楚kd树构造的原理
// 完成kd 树初始化的
// labels暂时不需要，因为我们用不到
struct tree_node {
  size_t id;     //
  size_t split;  // 切割的方向轴线序号应该是其中某一个feature
  tree_node *left, *right;
  // tree_node():left(NULL), right(NULL), id(-1), split(-1) {}
};
template <typename T>
struct tree_model {
  tree_node *root;
  const T *datas;     // 数据的集合
  size_t n_samples;   // 数据的数量, x坐标
  size_t n_features;  // 数据的维度， y坐标
};

// void free_tree_memory(tree_node *root);
// 全局函数
template <typename T>
tree_model<T> *build_kdtree(const T *datas, size_t rows, size_t cols);
// template<typename T>
// T* k_nearests_neighbor(const tree_model<T> *model, const T *x_test, size_t
// len, size_t k, bool clf);

// template<typename T>
// void find_k_nearests(const tree_model<T> *model, const T *coor, size_t k,
// size_t *args, T *dists);

#endif