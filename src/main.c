//Ders - 70
//Malloc - Calloc - Realloc - Free/ Dinamik hafıza erişimi
//malloc -> Türden bağımsızdır (void)
//malloc -> start address of heap memory block
//ex1
/*
#include <cstdio>
#include <cstdlib>

void *checkedMalloc(size_t n) {
    void *pd = (int *)malloc(sizeof(int) * n);
    if(!pd) {
        printf("Cannot allocate memory\n");
        exit(EXIT_FAILURE);
    }
    return pd;
}

int main() {
    int n;
    int *pd;
    int typeValue = 0;

    printf("How many elements do you require? :");
    scanf(%d,&n);

    pd = (int *)checkedMalloc(n);
    
    for(int i=0; i<n; i++) {
        //pd[i] = i;
        *(pd + i) = i;
    }
    
    for(int i=0; i<n; i++) {
        printf("pd[%d] : %d ", i, *(pd + i));
    }
    printf("\n");

    return 0;
}
*/
//Free function
//
//ex2
#include <cstdio>
#include <cstdlib>

void *checkedMalloc(size_t n) {
    void *pd = (int *)malloc(sizeof(int) * n);
    if(!pd) {
        printf("Cannot allocate memory\n");
        exit(EXIT_FAILURE);
    }
    return pd;
}

int main() {
    int n;
    int *pd;
    int typeValue = 0;

    printf("How many elements do you require? :");
    scanf(%d,&n);

    pd = (int *)checkedMalloc(n);
    
    for(int i=0; i<n; i++) {
        //pd[i] = i;
        *(pd + i) = i;
    }
    
    for(int i=0; i<n; i++) {
        printf("pd[%d] : %d ", i, *(pd + i));
    }
    printf("\n");

    return 0;
}





