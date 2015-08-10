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
//@brief   digitalFilter.h - class to create IIR and FIR digital filter
//         coefficients in the Matlab (2010) style. This style being vectors
//         of coefficients for the numberator (b) and denominator (a) 
//         respectively. 
//         Please refer to the matlab documentation page for implementation
//         details: http://www.mathworks.com/access/helpdesk/help/techdoc/ref/filter.html

#ifndef _DIGITALFILTER_H_
#define _DIGITALFILTER_H_

#include <vector>

class DigitalFilter
{

public:
	// Constructors
	DigitalFilter(int filterOrder_userdef, bool isIIR);
	DigitalFilter(int filterOrder_userdef, bool isIIR, const std::vector<double> &b_userdef, const std::vector<double> &a_userdef);
	~DigitalFilter(void);	// Destructicon

	double getNextFilteredValue(double u_current);
	void  reset(double reset_value);

	bool setCoefficients(const std::vector<double> &b_userdef, const std::vector<double> &a_userdef);
	bool isInitialized() const;

protected:
	std::vector<double> a_, b_;		// filter coefficients
	std::vector<double> u_, x_;		// filter input and output states
	
private:
	int filterOrder_;
	bool IIR_;
	bool initialized_;
	
};

#endif

