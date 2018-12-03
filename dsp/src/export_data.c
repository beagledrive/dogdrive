#include <stdio.h>
int write_to_file(float Valpha,float Vbeta,float Ia,float Ib,float Ic,float Wr,float Te_cur);

int write_to_file(float Valpha,float Vbeta,float Ia,float Ib,float Ic,float Wr,float Te_cur, char const *fileName)
{
   FILE *f = fopen(fileName, "w");
   if (f == NULL) return -1;
   fprintf(f, "%f,%f,%f,%f,%f,%f,%f\n",Valpha,Vbeta,Ia,Ib,Ic,Wr,Te_cur);
   fclose(f);
   return 0;
}
