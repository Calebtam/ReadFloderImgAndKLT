#include <iostream> 
#include <queue> 
using namespace std; 
  
int main() 
{ 
  
    queue<int, char> mp = { 
        { 1, 'a'}, 
        { 2, 'b'}, 
        { 3, 'c'}, 
        { 4, 'd'}, 
        { 5, 'e'}, 
        { 0, 'f'}, 
    }; 

    
    // cout << "Map contains "
    //      << "following elements in"
    //      << " reverse order "
    //      << mp.rbegin()->first  << " "
    //      << mp.rbegin()->second
    //      << endl;
  
    // for (auto i = mp.rbegin(); i != mp.rend(); ++i) { 
  
    //     cout << i->first 
    //          << " = "
    //          << i->second 
    //          << endl; 
    // } 
  
    return 0; 
}