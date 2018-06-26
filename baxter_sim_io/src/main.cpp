/*********************************************************************
 # Copyright (c) 2013-2015, Rethink Robotics
 # All rights reserved.
 #
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions are met:
 #
 # 1. Redistributions of source code must retain the above copyright notice,
 #    this list of conditions and the following disclaimer.
 # 2. Redistributions in binary form must reproduce the above copyright
 #    notice, this list of conditions and the following disclaimer in the
 #    documentation and/or other materials provided with the distribution.
 # 3. Neither the name of the Rethink Robotics nor the names of its
 #    contributors may be used to endorse or promote products derived from
 #    this software without specific prior written permission.
 #
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 # AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 # IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 # ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 # LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 # CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 # SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 # INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 # CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 # ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 # POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/**
 *  \author Hariharasudan Malaichamee
 */
#include <QtGui>
#include <QApplication>
#include <baxter_sim_io/baxter_io.hpp>
#include <baxter_sim_io/qnode.hpp>
#include <signal.h>
#include <sys/types.h>

void signalhandler(int sig)
{
  if (sig == SIGINT)
  {
    qApp->quit();
  }
  else if (sig == SIGTERM)
  {
    qApp->quit();
  }
}

int main(int argc, char** argv)
{
  QApplication app(argc, argv);
  baxter_sim_io::BaxterIO w(argc, argv);
  // Register Signal handler for ctrl+c
  signal(SIGINT, signalhandler);
  signal(SIGTERM, signalhandler);
  w.show();
  app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));

  int result = app.exec();
  return result;
}

baxter_core_msgs::NavigatorState baxter_sim_io::QNode::left_arm_nav, baxter_sim_io::QNode::right_arm_nav,
    baxter_sim_io::QNode::left_shoulder_nav, baxter_sim_io::QNode::right_shoulder_nav;

baxter_core_msgs::DigitalIOState baxter_sim_io::QNode::left_cuff_squeeze, baxter_sim_io::QNode::right_cuff_squeeze,
    baxter_sim_io::QNode::left_cuff_ok, baxter_sim_io::QNode::right_cuff_ok, baxter_sim_io::QNode::left_cuff_grasp,
    baxter_sim_io::QNode::right_cuff_grasp, baxter_sim_io::QNode::left_shoulder, baxter_sim_io::QNode::right_shoulder;
