#include <stdio.h>

#define VEC 1
#define STATES 2


double A[STATES][STATES] = {{1,0},{0,1}};
double B[STATES][VEC] = {{2},{5}};
double C[STATES][VEC];

void print_vec(double A[STATES][VEC]) {
  for(int i=0;i<STATES;i++) {
      printf("%.0f\n",A[i][VEC-1]);
  }
}

double* matmul_vec(double A[STATES][STATES], double B[STATES][VEC],double* C) {
    double temp[STATES][VEC];
    for(int i=0;i<STATES;i++) {
      int sum=0;
      for(int j=0;j<STATES;j++) {
        sum += A[i][j]*B[j][VEC-1];
      }
      C[i][VEC-1] = sum;
    }
    return &C;
}

// double* matmul_square() {
//   for(int i=0;i<STATES;i++) {
//       printf("%d\n",A[i][VEC-1]);
//   }
//
// }

int main(void) {

  matmul_vec(A,B);



  return 0;
}
