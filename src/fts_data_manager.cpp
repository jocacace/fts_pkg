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

#include "fts_data_manager.h"

void load_param( double & p, double def, string name ) {
	ros::NodeHandle n_param("~");
	if( !n_param.getParam( name, p))
		p = def;
	cout << name << ": " << "\t" << p << endl;
}

void load_param( string & p, string def, string name ) {
	ros::NodeHandle n_param("~");
	if( !n_param.getParam( name, p))
		p = def;
	cout << name << ": " << "\t" << p << endl;
}

void load_param( int & p, int def, string name ) {
	ros::NodeHandle n_param("~");
	if( !n_param.getParam( name, p))
		p = def;
	cout << name << ": " << "\t" << p << endl;
}

void load_param( bool & p, bool def, string name ) {
	ros::NodeHandle n_param("~");
	if( !n_param.getParam( name, p))
		p = def;
	cout << name << ": " << "\t" << p << endl;
}

void load_param( XmlRpc::XmlRpcValue & p,  XmlRpc::XmlRpcValue  def, string name ) {
	ros::NodeHandle n_param("~");
	if( !n_param.getParam( name, p))
		p = def;
	cout << name << ": " << "\t" << p << endl;
}

fts_data::fts_data() {

	_f = Zeros;
	_t = Zeros;
	_fts_itr = 0;

	_mean_f_bias = Zeros;
	_mean_t_bias = Zeros;
	_to_apply_bias = true;

	_R_f = Identity;
	_R_t = Identity;

	XmlRpc::XmlRpcValue identity; 
	identity.setSize(9);
	identity[0] = 1.0; identity[1] = 0.0; identity[2] = 0.0;
	identity[3] = 0.0; identity[4] = 1.0; identity[5] = 0.0;
	identity[6] = 0.0; identity[7] = 0.0; identity[8] = 1.0;
	XmlRpc::XmlRpcValue rotation_force;
	XmlRpc::XmlRpcValue rotation_torque;	
	load_param( rotation_force, identity, "rotation_force");
	load_param( rotation_torque, identity, "rotation_torque");
	
	assert(rotation_force.getType() == XmlRpc::XmlRpcValue::TypeArray);
	assert(rotation_torque.getType() == XmlRpc::XmlRpcValue::TypeArray);

	Fill(_R_f ) = rotation_force[0], rotation_force[1], rotation_force[2],
								rotation_force[3], rotation_force[4], rotation_force[5],
								rotation_force[6], rotation_force[7], rotation_force[8];
	Fill(_R_t ) = rotation_torque[0], rotation_torque[1], rotation_torque[2],
								rotation_torque[3], rotation_torque[4], rotation_torque[5],
								rotation_torque[6], rotation_torque[7], rotation_torque[8];


	load_param( _fts_f_th[0], 4.0, "fts_f_x_th" );
	load_param( _fts_f_th[1], 4.0, "fts_f_y_th" );
	load_param( _fts_f_th[2], 4.0, "fts_f_z_th" );	
	load_param( _fts_t_th[0], 4.0, "fts_t_x_th" );
	load_param( _fts_t_th[1], 4.0, "fts_t_y_th" );
	load_param( _fts_t_th[2], 4.0, "fts_t_z_th" );
	load_param(_fts_topic_in, "fts_data", "fts_topic_in" );
	load_param(_fts_topic_out, "fts", "fts_topic_out" );
  load_param(_bias_itr, 200, "bias_itr" );
  load_param(_force_from_joy, false, "force_from_joy");

  if( !_force_from_joy )
  	_fts_sub = _nh.subscribe(_fts_topic_in.c_str(), 0, &fts_data::fts_cb, this );
  else
  	_joy_sub = _nh.subscribe( "/joy", 0, &fts_data::joy_cb, this);

  _fts_pub = _nh.advertise<geometry_msgs::Wrench>(_fts_topic_out.c_str(), 0);	
	

}

void fts_data::joy_cb( sensor_msgs::Joy joy ) {
	_joy_forces = makeVector( joy.axes[1], joy.axes[0], joy.axes[3]);
}


void fts_data::force_from_joy() {


	ros::Rate r(100);
	
	while(ros::ok()) {

		_f = makeVector( _joy_forces[0], -_joy_forces[1], -_joy_forces[2] );
		_f*=7.0;
		_t = Zeros;

		_fts_out.force.x = _f[0];
		_fts_out.force.y = _f[1];
		_fts_out.force.z = _f[2];
		_fts_out.torque.x = _t[0];
		_fts_out.torque.y = _t[1];
		_fts_out.torque.z = _t[2];

		_fts_pub.publish( _fts_out );				
		r.sleep();
	}


}

void fts_data::fts_cb( geometry_msgs::WrenchStamped wrench ) {

	
	//Raw data
	Vector<3> f = makeVector ( wrench.wrench.force.x, wrench.wrench.force.y, wrench.wrench.force.z );
	Vector<3> t = makeVector ( wrench.wrench.torque.x, wrench.wrench.torque.y, wrench.wrench.torque.z );
	_f = f;
	_t = t;

	//---Bias calculation
	if( _fts_itr++ < _bias_itr ) {
		_fts_f_bias.push_back( f );
		_fts_t_bias.push_back( t );		
	} //Fill bias
	else {

		if( _to_apply_bias ) {
		
			for(int i=0; i<_fts_f_bias.size(); i++ ) {
				_mean_f_bias +=	_fts_f_bias[i];
				_mean_t_bias += _fts_t_bias[i];
			}	
			_mean_f_bias /= _fts_f_bias.size();				
			_mean_t_bias /= _fts_t_bias.size();
			_to_apply_bias = false;
		} //calculte bias
		//cout << "mean bias: " << _mean_f_bias << endl;
		//Apply bias
		_f -= _mean_f_bias;
		_t -= _mean_t_bias;

		for( int i=0; i<3; i++ ) {	
			if( fabs(_f[i]) > _fts_f_th[i] ) {
				if(_f[i] > 0.0)	
					_f[i] = _f[i] - _fts_f_th[i];
				else
					_f[i] = _f[i] + _fts_f_th[i];
			}
			else 
				_f[i] = 0.0;
		}	//Apply force threshold

		for( int i=0; i<3; i++ ) {	
			if( fabs(_t[i]) > _fts_t_th[i] ) {
				if(_t[i] > 0.0)	
					_t[i] = _t[i] - _fts_t_th[i];
				else
					_t[i] = _t[i] + _fts_t_th[i];
			}
			else 
				_t[i] = 0.0;
		}	//Apply torque threshold

		_f = _R_f*_f;
		_t = _R_t*_t;

		_fts_out.force.x = _f[0];
		_fts_out.force.y = _f[1];
		_fts_out.force.z = _f[2];
		_fts_out.torque.x = _t[0];
		_fts_out.torque.y = _t[1];
		_fts_out.torque.z = _t[2];

		_fts_pub.publish( _fts_out );		
	}
}

void fts_data::run() {
	
	if( _force_from_joy )
		boost::thread force_from_joy_t( &fts_data::force_from_joy, this);

	ros::spin();
}

int main( int argc, char** argv ) {

	ros::init( argc, argv, "fts_data");
	fts_data data;
	data.run();
	return 0;
}