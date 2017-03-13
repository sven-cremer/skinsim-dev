/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, UT Arlington
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
 *   * Neither the name of UT Arlington nor the names of its
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
 *********************************************************************/

/* Author: Isura Ranatunga
 *         Sven Cremer
 *
 * auto_experimenter.cpp
 *  Created on: Jul 21, 2014
 */

#include <experimenter/src/skinsim_testing_framework.cc>

// Main
int main(int argc, char** argv)
{

	std::string exp_name = "exp01";		// Default experiment name

	// Check the number of command-line parameters
	if (argc == 2)
	{
		exp_name = argv[1];				// Set experiment name
	}
	else
	{
		std::cout<<"\n\tUsage: "<<argv[0]<<" [EXPERIMENT NAME]\n\n";
	}

	// Save current time
	boost::chrono::steady_clock::time_point start = boost::chrono::steady_clock::now();

	// Run simulation
	SkinSimTestingFramework skinSimTestingFrameworkObject;
	skinSimTestingFrameworkObject.runTests(exp_name, false);

	// Print time elapsed
	boost::chrono::duration<double> sec = boost::chrono::steady_clock::now() - start;
	std::cout << "\n -> Auto experimenter completed in " << sec.count() << " seconds.\n\n";

	return 0;

}

