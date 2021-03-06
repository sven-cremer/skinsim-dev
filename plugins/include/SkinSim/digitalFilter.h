/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * As long as you retain this notice you can do whatever you want with this 
 * stuff. If we meet some day, and you think this stuff is worth it, you can 
 * buy me a beer in return.
 * - Joe Romano and Will McMahan
 * ----------------------------------------------------------------------------
 */
//@author  Joe Romano
//@author  Will McMahan
//@email   joeromano@gmail.com
//@brief   digitalFilter.h - class to create IIR and FIR digital filter
//         coefficients in the Matlab (2010) style. This style being vectors
//         of coefficients for the numberator (b) and denominator (a) 
//         respectively. 
//         Please refer to the matlab documentation page for implementation
//         details: http://www.mathworks.com/access/helpdesk/help/techdoc/ref/filter.html

/*
 *  Modified on: Feb 4, 2016
 *       Author: Sven Cremer
 *  Description: Using Eigen instead of dynamic arrays, new constructor and init functions
 */

#ifndef _DIGITALFILTER_H_
#define _DIGITALFILTER_H_

#include <Eigen/Core>

namespace ice{

class digitalFilter
{

public:
	// Constructors
	digitalFilter();
	digitalFilter(int filterOrder_userdef, bool isIIR);	
	digitalFilter(int filterOrder_userdef, bool isIIR, double *b_userdef, double *a_userdef);

	~digitalFilter(void);	// Destructor

	bool init(int filterOrder_userdef, bool isIIR, Eigen::VectorXd b_userdef, Eigen::VectorXd a_userdef);
	double getNextFilteredValue(double u_current);
	bool initialized;

protected:
	Eigen::VectorXd a, b;		// filter coefficients
	Eigen::VectorXd u, x;		// filter input and output states
	
private:
	int filterOrder;
    bool IIR;
	
};

} // namespace

#endif

