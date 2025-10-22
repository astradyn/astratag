#include "quadrilateral.hpp"
#include <iostream>

int main() {
    try {
        const std::string inputFolder = "test_images";
        const std::string outputFolder = "results";
        
        processImageBatch(inputFolder, outputFolder);
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}