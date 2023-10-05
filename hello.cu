#include <iostream>

__global__ void myKernel(void) { }

int main(void) {
    myKernel <<<1, 1>>>();
    std::cout << "Hello CUDA world\n";
}
