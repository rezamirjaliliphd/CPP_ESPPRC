#include <iostream>
#include <vector>
#include <unordered_set>
#include <algorithm>

int main() {
   // Test cases
   std::vector<std::pair<std::vector<int>, std::vector<int>>> test_cases = {
       {{1, 2, 3, 4}, {4, 5, 6, 7}},     // Valid case: only 4 is common
       {{1, 2, 3, 4}, {5, 6, 7, 8}},     // Invalid: no common element
       {{1, 2, 3, 4}, {4, 2, 5, 6}},     // Invalid: multiple common elements (2 and 4)
       {{}, {1, 2, 3}},                   // Invalid: first vector empty
       {{1, 2, 3}, {}},                   // Invalid: second vector empty
   };

   auto checkVectors = [](const std::vector<int>& vec1, const std::vector<int>& vec2) {
       if (vec1.empty() || vec2.empty() || vec1.back() != vec2.front()) {
           return false;
       }
       
       std::unordered_set<int> set1(vec1.begin(), vec1.end() - 1);
       return std::none_of(vec2.begin() + 1, vec2.end(), 
           [&set1](const int& x) { return set1.count(x) > 0; });
   };

   // Test each case
   for (size_t i = 0; i < test_cases.size(); ++i) {
       const auto& test = test_cases[i];
       bool result = checkVectors(test.first, test.second);
       
       // Print vectors
       std::cout << "Test case " << i + 1 << ":\n";
       std::cout << "Vector 1: ";
       for (const auto& num : test.first) {
           std::cout << num << " ";
       }
       std::cout << "\nVector 2: ";
       for (const auto& num : test.second) {
           std::cout << num << " ";
       }
       std::cout << "\nResult: " << (result ? "Valid" : "Invalid") << "\n\n";
   }

   return 0;
}