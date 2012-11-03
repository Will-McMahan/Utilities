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

import numpy

class DigitalFilter:

  def __init__(self,isIIR,b_userdef,a_userdef):
      self.b = numpy.array(b_userdef)
      self.a = numpy.array(a_userdef)
      self.x = numpy.zeros(len(self.b))
      self.u = numpy.zeros(len(self.b))
      self.IIR = isIIR

  def getNextFilteredValue(self,u_current):
      # Shift x and u vectors, losing the last elements
      #and putting new u value in zeroth spot
      self.x[1:] = self.x[0:-1]
      self.u = numpy.concatenate((numpy.array([u_current]), self.u[0:-1]))

      # Simulate the system
      output = self.b[0] * self.u[0]

      # if this is an IIR filter
      if(self.IIR):
        for i in range(len(self.b))[1:]:
          output = output + self.b[i] * self.u[i] - self.a[i] * self.x[i]
      else:
        for i in range(len(self.b))[1:]:
          output = output + self.b[i] * self.u[i]

      self.x[0] = output
      return output

  def reset(self,reset_value):
      self.x = numpy.ones(len(self.x)) * reset_value
      self.u = numpy.ones(len(self.u)) * reset_value
