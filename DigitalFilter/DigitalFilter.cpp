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
	filterOrder_ = filterOrder_userdef;
	IIR_ = isIIR;
	
	// Initialize the vectors with zeros
	b_.assign(filterOrder_ + 1, 0.0);
	a_.assign(filterOrder_ + 1, 0.0);

	x_.assign(filterOrder_ + 1, 0.0);
	u_.assign(filterOrder_ + 1, 0.0);
}	

DigitalFilter::DigitalFilter(int filterOrder_userdef, bool isIIR, const std::vector<double> &b_userdef, const std::vector<double> &a_userdef)
{
	filterOrder_ = filterOrder_userdef;
	IIR_ = isIIR;

	// Initialize the vectors with zeros
	x_.assign(filterOrder_ + 1, 0.0);
	u_.assign(filterOrder_ + 1, 0.0);

	// Copy the filter coefficients.
	setCoefficients(b_userdef, a_userdef);
}

DigitalFilter::~DigitalFilter(void)
{

}
bool DigitalFilter::isInitialized() const
{
	return initialized_;
}

bool DigitalFilter::setCoefficients(const std::vector<double> &b_userdef, const std::vector<double> &a_userdef)
{
	// Initialize the vectors with zeros
	b_.assign(b_userdef.begin(), b_userdef.end());
	a_.assign(a_userdef.begin(), a_userdef.end());

	if ((b_.size() == filterOrder_ + 1) && (a_.size() == filterOrder_ + 1)
		&& (x_.size() == filterOrder_ + 1) && (u_.size() == filterOrder_ + 1) ) {
		initialized_ = true;
	}
	else {
		initialized_ = false;
	}

	return initialized_;
}

double DigitalFilter::getNextFilteredValue(double u_current)
{
	if (!initialized_) {
		return 0.0;
	}

	/* Shift x2 and u2 vectors, losing the last elements and putting new u2 value in zeroth spot. */
	for (int i = filterOrder_ ; i > 0 ; i--) {
		x_[i] = x_[i-1];
		u_[i] = u_[i-1];
	}
	u_[0] = u_current; 

	/* Simulate system. */
	double output = b_[0] * u_[0];
	  
  // if we have an IIR filter
  if(IIR_)
  {
    for (int i = 1 ; i < (filterOrder_+1) ; i++) {
      output += b_[i] * u_[i] - a_[i] * x_[i];
    }
  }

 // if we have an FIR filter
  else
  {
    for (int i = 1 ; i < (filterOrder_+1) ; i++) {
      output += b_[i] * u_[i];
    }
  }

	/* Put the result in shared memory and in the x2 vector. */
	x_[0] = output;

	return output;
}

void  DigitalFilter::reset(double reset_value)
{
	x_.assign(filterOrder_ + 1, reset_value);
	u_.assign(filterOrder_ + 1, reset_value);
}