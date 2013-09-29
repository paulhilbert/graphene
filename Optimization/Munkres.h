#ifndef MUNKRES_H
#define MUNKRES_H

// Adapted from Dominic Battre's implementation of the Hungarian algorithm by hbc@mit.edu

#include <Data/RawArray2D.h>

#define INF 100000

namespace Optimization {

template <class CostType>
class Munkres {
protected:
	typedef unsigned int uint;

public:
	Munkres(uint size) {
		parent_row   = new long[size];
		unchosen_row = new long[size];
		row_dec      = new CostType[size];
		col_inc      = new CostType[size];
		slack        = new CostType[size];
		slack_row    = new long[size];
	}

	virtual ~Munkres() {
		delete[] parent_row;
		delete[] unchosen_row;
		delete[] row_dec;
		delete[] col_inc;
		delete[] slack;
		delete[] slack_row;
	}

	// There are good reasons for copying costs here. The costs matrix
	// still needs to be read in EditDistance after solving.
	void solve(::Data::RawArray2D<CostType> costs, int size, long* col_mate, long* row_mate);

protected:
	long* parent_row;
	long* unchosen_row;
	CostType* row_dec;
	CostType* col_inc;
	CostType* slack;
	long* slack_row;
};

template <class CostType>
void Munkres<CostType>::solve(::Data::RawArray2D<CostType> costs, int size, long* col_mate, long* row_mate) {
	long m=size,n=size;
	long k;
	long l;
	long j;
	CostType s;
	//long*parent_row;
	//long*unchosen_row;
	long t;
	long q;
	//CostType*row_dec;
	//CostType*col_inc;
	//CostType*slack;
	//long*slack_row;
	long unmatched;

	memset(col_mate,     0, sizeof(long)*n);
	memset(row_mate,     0, sizeof(long)*n);

	memset(parent_row,   0, sizeof(long)*n);
	memset(unchosen_row, 0, sizeof(long)*m);
	memset(row_dec,      0, sizeof(CostType)*m);
	memset(col_inc,      0, sizeof(CostType)*n);
	memset(slack,        0, sizeof(CostType)*n);
	memset(slack_row,    0, sizeof(long)*n);

	/*
	for (i=0; i < n; ++i) col_mate[i]=0;
	for (i=0; i < n; ++i) row_mate[i]=0;
	for (i=0; i < n; ++i) parent_row[i]=0;
	for (i=0;i < m; ++i) unchosen_row[i]=0;
	for (i=0;i<m;++i) row_dec[i]=0;
	for (i=0;i<n;++i) col_inc[i]=0;
	for (i=0; i < n; ++i) slack[i]=0;
	for (i=0; i < n; ++i) slack_row[i]=0;
	*/

	// Do heuristic
	for(l = 0; l < n; l++)
	{
		s = costs(0, l);
		for(k = 1; k < n; k++)
			if(costs(k, l)<s)s= costs(k, l);
		if(s!=0)
		for(k = 0;k<n;k++)
			costs(k, l)-= s;
	}

	t= 0;
	for(l= 0;l<n;l++)
	{
		row_mate[l]= -1;
		parent_row[l]= -1;
		col_inc[l]= 0;
		slack[l]= INF;
	}
	for(k= 0;k<m;k++)
	{
		s= costs(k, 0);
		for(l= 1;l<n;l++)
			if(costs(k, l)<s) s=costs(k, l);
		row_dec[k]= s;
		for(l= 0;l<n;l++)
			if((s==costs(k, l))&&(row_mate[l]<0))
			{
				col_mate[k]= l;
				row_mate[l]= k;
				goto row_done;
			}
		col_mate[k]= -1;
		unchosen_row[t++]= k;
		row_done:;
	}

	if(t==0)goto done;
	unmatched= t;
	while(1)
	{
		q = 0;
		while(1)
		{
			while(q<t)
			{
				{
					k= unchosen_row[q];
					s= row_dec[k];
					for(l= 0;l<n;l++)
						if(slack[l])
						{
							CostType del;
							del = costs(k, l)-s+col_inc[l];
							if(del<slack[l])
							{
								if(del==0)
								{
									if(row_mate[l]<0)goto breakthru;
									slack[l]= 0;
									parent_row[l]= k;
									unchosen_row[t++]= row_mate[l];
								}
								else
								{
									slack[l]= del;
									slack_row[l]= k;
								}
							}
						}
				}
				q++;
			}

			s= INF;
			for(l= 0;l<n;l++)
				if(slack[l]&&slack[l]<s)
			s= slack[l];
			for(q= 0;q<t;q++)
				row_dec[unchosen_row[q]]+= s;
			for(l= 0;l<n;l++)
				if(slack[l])
				{
					slack[l]-= s;
					if(slack[l]==0)
					{
						k= slack_row[l];
						if(row_mate[l]<0)
						{
							for(j= l+1;j<n;j++)
							if(slack[j]==0)col_inc[j]+= s;
							goto breakthru;
						}
						else
						{
							parent_row[l]= k;
							unchosen_row[t++]= row_mate[l];
						}
					}
				}
				else col_inc[l]+= s;
		 }
breakthru:
  while(1){
   j= col_mate[k];
   col_mate[k]= l;
   row_mate[l]= k;
   if(j<0)break;
   k= parent_row[j];
   l= j;
  }
  if(--unmatched==0)goto done;
  t= 0;
  for(l= 0;l<n;l++){
   parent_row[l]= -1;
   slack[l]= INF;
  }
  for(k= 0;k<m;k++)
   if(col_mate[k]<0){
    unchosen_row[t++]= k;
   }
}
	done:

		for (k=0;k<m;++k)
		{
		  for (l=0;l<n;++l)
		  {
		   costs(k, l)=costs(k, l)-row_dec[k]+col_inc[l];
		  }
		}
		//delete[] parent_row;
		//delete[] unchosen_row;
		//delete[] row_dec;
		//delete[] col_inc;
		//delete[] slack;
		//delete[] slack_row;
	}

}

#endif
