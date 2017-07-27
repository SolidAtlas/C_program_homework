//function to create a power table
//author: Justin Garcia

#include <stdio.h>
#include <math.h>

void powertable (int first, int last,double d);
 
int main (void)
{
int first, last;
    double d;
    printf("enter two numbers and exponent\n");
    scanf("%i,%i,%lf", &first,&last, &d);
    powertable(first,last,d);
   

return 0;
}
void powertable (int first, int last,double d){
    for(;first<=last;first++){
        
    printf("%i \t %.2lf \n", first,pow(first,d));
    }  
}
