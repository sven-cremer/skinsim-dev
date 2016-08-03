/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <joeromano@gmail.com> wrote this file. As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return
 * Joe Romano and Will McMahan
 * ----------------------------------------------------------------------------
 */
//@author  Joe Romano
//@author  Will McMahan
//@email   joeromano@gmail.com
//@brief   digitalFilter.cpp - class to create IIR and FIR digital filter
//         coefficients in the Matlab (2010) style. This style being vectors
//         of coefficients for the numberator (b) and denominator (a) 
//         respectively. 
//         Please refer to the matlab documentation page for implementation
//         details: http://www.mathworks.com/access/helpdesk/help/techdoc/ref/filter.html

#include "SkinSim/digitalFilter.h"

using namespace ice;

digitalFilter::digitalFilter()
{
	initialized = false;
}

digitalFilter::digitalFilter(int filterOrder_userdef, bool isIIR)
{
	filterOrder = filterOrder_userdef;
    IIR = isIIR;
	
    b.resize(filterOrder + 1);
    a.resize(filterOrder + 1);

    x.resize(filterOrder + 1);
    u.resize(filterOrder + 1);

	// Initialize the arrays with zeros
	b.setZero();
	a.setZero();
	x.setZero();
	u.setZero();

	initialized = true;
}	

digitalFilter::digitalFilter(int filterOrder_userdef, bool isIIR, double *b_userdef, double *a_userdef)
{

	filterOrder = filterOrder_userdef;
        IIR = isIIR;

	b.resize(filterOrder + 1);
	a.resize(filterOrder + 1);

	x.resize(filterOrder + 1);
	u.resize(filterOrder + 1);

	// Initialize the arrays

	for(int i = 0; i < (filterOrder + 1); i++)
	{
		b[i] = b_userdef[i];
		a[i] = a_userdef[i];
		x[i] = 0.0;
		u[i] = 0.0;
	}
}

bool digitalFilter::init(int filterOrder_userdef, bool isIIR, Eigen::VectorXd b_userdef, Eigen::VectorXd a_userdef)
{

//	std::cout<<"FILTER: "<<a_userdef<<"\nSIZE:"<<a_userdef.size()<<"\n";
	if(a_userdef.size() != filterOrder_userdef + 1)
		return false;

	if(b_userdef.size() != filterOrder_userdef + 1)
		return false;

	filterOrder = filterOrder_userdef;
    IIR = isIIR;

	b.resize(filterOrder + 1);
	a.resize(filterOrder + 1);

	x.resize(filterOrder + 1);
	u.resize(filterOrder + 1);

	// Initialize the arrays
	b = b_userdef;
	a = a_userdef;
	x.setZero();
	u.setZero();

	initialized = true;

	return true;
}


double digitalFilter::getNextFilteredValue(double u_current)
{
	/* Shift x2 and u2 vectors, losing the last elements and putting new u2 value in zeroth spot. */
	for (int i = filterOrder ; i > 0 ; i--) {
		x[i] = x[i-1];
		u[i] = u[i-1];
	}
	u[0] = u_current; 

	/* Simulate system. */
	double output = b[0] * u[0];
	  
        // if we have an IIR filter            
        if(IIR)
        {
            for (int i = 1 ; i < (filterOrder+1) ; i++) {
                    output += b[i] * u[i] - a[i] * x[i];
            }
        }

       // if we have an FIR filter
       else
       {
            for (int i = 1 ; i < (filterOrder+1) ; i++) {
                    output += b[i] * u[i];
            }
       }

	/* Put the result in shared memory and in the x2 vector. */
	x[0] = output;

	return output;
}

digitalFilter::~digitalFilter(void)
{

}
