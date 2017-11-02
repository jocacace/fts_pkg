/*
 * Copyright (C) 2017, Jonathan Cacace and Prisma Lab
 * Email id : jonathan.cacace@gmail.com
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
*/

#include "ros/ros.h"
#include "TooN/TooN.h"
#include "robohelper/robohelper.hpp"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Wrench.h"
#include "sensor_msgs/Joy.h"
#include "boost/thread.hpp"

using namespace std;
using namespace TooN;

class fts_data {
public:
    fts_data();
    void fts_cb( geometry_msgs::WrenchStamped wrench );
    void run();
    void joy_cb( sensor_msgs::Joy joy );
    void force_from_joy();

private:

    ros::NodeHandle _nh;
    ros::Subscriber _fts_sub;
    ros::Subscriber _joy_sub;
    ros::Publisher  _fts_pub;
    Vector<3> _t;
    Vector<3> _f;

    Vector<3> _mean_f_bias;
    Vector<3> _mean_t_bias;

    //---Params
    string _fts_topic_in;
    string _fts_topic_out;    
    double _fts_t_th[3];
    double _fts_f_th[3];
    int _bias_itr;
    bool _force_from_joy;
    //---

    bool _to_apply_bias;
    int _fts_itr;
    vector< Vector<3> > _fts_f_bias;
    vector< Vector<3> > _fts_t_bias;

    Matrix<3> _R_f;
    Matrix<3> _R_t;
    
    Vector<3> _joy_forces;
    geometry_msgs::Wrench _fts_out;
};
