/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Joe Romano & Will McMahan
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the authors nor the names of the
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
//@author  Joe Romano
//@author  Will McMahan
//@email   joeromano@gmail.com
//@brief   digitalFilter.cpp - class to create IIR and FIR digital filter
//         coefficients in the Matlab (2010) style. This style being vectors
//         of coefficients for the numberator (b) and denominator (a) 
//         respectively. 
//         Please refer to the matlab documentation page for implementation
//         details: http://www.mathworks.com/access/helpdesk/help/techdoc/ref/filter.html

#include "DigitalFilter.h"

DigitalFilter::DigitalFilter(int filterOrder_userdef, bool isIIR)
{
	filterOrder = filterOrder_userdef;
        IIR = isIIR;
	
	b = new double [filterOrder + 1];
	a = new double [filterOrder + 1];

	x = new double [filterOrder + 1];
	u = new double [filterOrder + 1];

	// Initialize the arrays with zeros
	for(int i = 0; i < (filterOrder + 1); i++)
	{
		b[i] = 0.0;
		a[i] = 0.0;
		x[i] = 0.0;
		u[i] = 0.0;
	}
}	

DigitalFilter::DigitalFilter(int filterOrder_userdef, bool isIIR, double *b_userdef, double *a_userdef)
{
  
	filterOrder = filterOrder_userdef;
        IIR = isIIR;
	
	b = new double [filterOrder + 1];
	a = new double [filterOrder + 1];

	x = new double [filterOrder + 1];
	u = new double [filterOrder + 1];

	// Initialize the arrays
	
	for(int i = 0; i < (filterOrder + 1); i++)
	{
		b[i] = b_userdef[i];
		a[i] = a_userdef[i];
		x[i] = 0.0;
		u[i] = 0.0;
	}
	
}

double DigitalFilter::getNextFilteredValue(double u_current)
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

void  DigitalFilter::reset(double reset_value)
{
  //iterate through all filter values and set equal to zero
  for(int i = filterOrder; i > -1; i--)
  {
    x[i] = reset_value;
    u[i] = reset_value;
  }
}

DigitalFilter::~DigitalFilter(void)
{
	delete[] x;
	delete[] u;
	delete[] a;
	delete[] b;
}