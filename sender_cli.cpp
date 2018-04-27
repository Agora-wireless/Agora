#include "packageSender.hpp"

int main(int argc, char const *argv[])
{
    if(argc > 3)
        PackageSender sender(strtol(argv[1], NULL, 10), strtol(argv[2], NULL, 10), strtol(argv[3], NULL, 10));
    else
        PackageSender sender(strtol(argv[1], NULL, 10), strtol(argv[2], NULL, 10));
    printf("send package\n");
    return 0;
}
