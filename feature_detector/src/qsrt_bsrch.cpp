/**********************************

qsrt_bsrch.c - routines for quicksort and binary search, used by
the ror algorithm.

**********************************/

int partition(double a[], int n)
{
   int i, j, incr = 0, decr = 1, swap;
   double Tmp;

   i = 0;
   j = n-1;

   while (i != j)
   {
      if (a[i] > a[j])
      {
         Tmp = a[i];
         a[i] = a[j];
         a[j] = Tmp;

         swap = incr;
         incr = decr;
         decr = swap;
      }

      i += incr;
      j -= decr;
   }

   return j;
}

void quickSort(double a[], int n)
{
   int pivotIndex;

   if (n > 1)
   {
      pivotIndex = partition(a, n);

      quickSort(a, pivotIndex);

      quickSort(&a[pivotIndex+1], (n-pivotIndex-1));
   }
}
     
int binarySearch(double key, double a[],int n)
{ 
  int high, i, low;
  for (low=(-1), high=n;  high-low > 1;)
  {
     i = (high+low) / 2;
     if (key <= a[i])  
	   high = i;
     else      
	   low  = i;
  }
  return(high);
}




