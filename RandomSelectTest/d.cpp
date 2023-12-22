#include <unordered_map>
#include <random>
#include <vector>
#include <algorithm>
#include <iostream>

using namespace std;

// Select random partial element in an unordered_map by C++， return the partial unordered_map
template<typename K, typename V>
std::unordered_map<K, V> select_random_partial_elements(const std::unordered_map<K, V>& map, size_t n) {
    std::unordered_map<K, V> result;
    std::vector<std::pair<K, V>> elements(map.begin(), map.end());
    std::sample(elements.begin(), elements.end(), std::inserter(result, result.end()), n, std::mt19937{std::random_device{}()});
    return result;
}

// Select random partial element in an vector by C++， return the partial vector
template<typename T>
std::vector<T> random_partial_vector(const std::vector<T>& vec, int num_elements) {
    std::vector<T> partial_vec;
    partial_vec.reserve(num_elements);
    std::random_device rd;
    std::mt19937 g(rd());
    std::sample(vec.begin(), vec.end(), std::back_inserter(partial_vec), num_elements, g);
    return partial_vec;
}

// Select random correspond index partial element in two correspond vector by C++， return the two correspond index partial vector
template<typename T>
std::pair<std::vector<T>, std::vector<T>> random_partial_corresponding_vectors(const std::vector<T>& vec1, const std::vector<T>& vec2, int num_elements) {
    std::pair<std::vector<T>, std::vector<T>> partial_vecs;
    partial_vecs.first.reserve(num_elements);
    partial_vecs.second.reserve(num_elements);
    std::vector<int> indices(vec1.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(indices.begin(), indices.end(), g);
    for (int i = 0; i < num_elements; ++i) {
        partial_vecs.first.push_back(vec1[indices[i]]);
        partial_vecs.second.push_back(vec2[indices[i]]);
    }
    return partial_vecs;
}
// the two correspond vector is not the same type
template<typename T1, typename T2>
std::pair<std::vector<T1>, std::vector<T2>> random_partial_corresponding_vectors(const std::vector<T1>& vec1, const std::vector<T2>& vec2, int num_elements) {
    std::pair<std::vector<T1>, std::vector<T2>> partial_vecs;
    partial_vecs.first.reserve(num_elements);
    partial_vecs.second.reserve(num_elements);
    std::vector<int> indices(vec1.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(indices.begin(), indices.end(), g);
    for (int i = 0; i < num_elements; ++i) {
        partial_vecs.first.push_back(vec1[indices[i]]);
        partial_vecs.second.push_back(vec2[indices[i]]);
    }
    return partial_vecs;
}
int main(int argc, char **argv)
{
    // std::unordered_map<int, std::string> my_map = {{1, "one"}, {2, "two"}, {3, "three"}, {4, "four"}, {5, "five"}};
    // size_t n = 3;
    // std::unordered_map<int, std::string> random_elements = select_random_partial_elements(my_map, n);

    // for(auto it=random_elements.begin();it!=random_elements.end();it++){
    //     cout<<it->first<< " " <<it->second<<endl;
    // }

    // std::vector<int> my_vec = {1, 2, 3, 4, 5};
    // std::vector<int> partial_vec = random_partial_vector(my_vec, 3);
    // for(auto it=random_elements.begin();it!=random_elements.end();it++){
    //     cout<<it->first<< " " <<it->second<<endl;
    // }

    std::vector<int> vec1 = {1, 2, 3, 4, 5};
    std::vector<int> vec2 = {6, 7, 8, 9, 10};
    std::pair<std::vector<int>, std::vector<int>> partial_vecs = random_partial_corresponding_vectors(vec1, vec2, 3);

	for(unsigned int i = 0; i < partial_vecs.first.size(); ++i)
	{
        std::cout << "partial_vecs " << i << " " << partial_vecs.first[i] << " " << partial_vecs.second[i] << std::endl;
	}


}