/*
 *  contact_point_estimation_node.cpp
 *
 *
 *  Created on: Jan 14, 2014
 *  Authors:   Francisco Viña
 *            fevb <at> kth.se
 */

/* Copyright (c) 2014, Francisco Viña, CVAP, KTH
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of KTH nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL KTH BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

// ? MODIFIED VERSION TO WORK WITH ROS2
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
// #include <ros/ros.h>
// #include <std_srvs/Empty.h>

#include <contact_point_estimation/ContactPointEstimator.h>
#include <contact_point_estimation/ContactPointEstimatorParams.h>
#include <contact_point_estimation/SurfaceNormalEstimator.h>
#include <contact_point_estimation/SurfaceNormalEstimatorParams.h>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <boost/thread.hpp>
#include <boost/bind.hpp>



class ContactPointEstimationNode : public rclcpp::Node
{
public:
	// ? MODIFIED VERSION TO WORK WITH ROS2
	ContactPointEstimationNode() : Node("contact_point_estimation_node"),
		m_received_ft(false),
		m_received_twist(false),
		m_run_estimator(false)
    // ros::NodeHandle n_;
	{
		// ? PARAMETERS TO WORK IN ROS2 
		// declare_parameter<double>("gamma_r");
		// declare_parameter<double>("kappa_r");
		// declare_parameter<double>("beta_r");
		// declare_parameter<std::vector<double>>("initial_r");
		// declare_parameter<double>("gamma_n");
		// declare_parameter<double>("beta_n");
		// declare_parameter<std::vector<double>>("initial_n");
		// declare_parameter<double>("cpe_update_frequency");
		// declare_parameter<double>("sne_update_frequency");
		declare_parameter<double>("gamma_r", 7.0);
		declare_parameter<double>("kappa_r", 0.0);
		declare_parameter<double>("beta_r", 0.2);
		declare_parameter<std::vector<double>>("initial_r", {0.2,0.2,0.1});
		declare_parameter<bool>("sim", true);
		declare_parameter<double>("gamma_n", 0.0);
		declare_parameter<double>("beta_n", 0.0);
		declare_parameter<std::vector<double>>("initial_n", {0.0084,-0.0577,0.9983});
		declare_parameter<double>("cpe_update_frequency", 650.0);
		declare_parameter<double>("sne_update_frequency", 150.0);
		
		// ? PUBLISHERS AND SUBSCRIBERS TO WORK IN ROS2
		topicPub_ContactPointEstimate_ = this->create_publisher<geometry_msgs::msg::PointStamped>("contact_point_estimate", 10);
		topicPub_SurfaceNormalEstimate_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("surface_normal_estimate", 10);

		// topicSub_FT_Sensor_Sim_ = this->create_subscription<geometry_msgs::msg::Wrench>(
		// 	"ft_sensor_sim",
		// 	10,
		// 	std::bind(&ContactPointEstimationNode::topicCallback_FT_Sensor_Sim, this, std::placeholders::_1));

		// topicSub_FT_compensated_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
		// 	"ft_compensated", 
		// 	10, 
		// 	std::bind(&ContactPointEstimationNode::topicCallback_FT_compensated, this, std::placeholders::_1));
		// topicSub_Twist_FT_Sensor_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
		// 	"twist_ft_sensor",
		// 	10,
		// 	std::bind(&ContactPointEstimationNode::topicCallback_Twist_FT_Sensor, this, std::placeholders::_1));

		// ? SERVICES TO WORK IN ROS2
		srvServer_Start_ = this->create_service<std_srvs::srv::Empty>(
			"start",
			std::bind(&ContactPointEstimationNode::srvCallback_Start, this, std::placeholders::_1, std::placeholders::_2));
		srvServer_Stop_ = this->create_service<std_srvs::srv::Empty>(
			"stop",
			std::bind(&ContactPointEstimationNode::srvCallback_Stop, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(get_logger(), "Contact Point Estimation Node (ROS2) ready");
	}

	// ContactPointEstimationNode()
	// {
	// 	n_ = ros::NodeHandle("~");
    //     m_received_ft = false;
    //     m_run_estimator = false;

    //     cpe_params = NULL;
    //     cpe = NULL;

    //     sne_params = NULL;
    //     sne = NULL;

	// 	topicPub_ContactPointEstimate_ = n_.advertise<geometry_msgs::PointStamped>("contact_point_estimate", 1);
	// 	topicPub_SurfaceNormalEstimate_ = n_.advertise<geometry_msgs::Vector3Stamped>("surface_normal_estimate", 1);

    //     topicSub_FT_compensated_ = n_.subscribe("ft_compensated", 1, &ContactPointEstimationNode::topicCallback_FT_compensated, this);

    //     topicSub_Twist_FT_Sensor_ = n_.subscribe("twist_ft_sensor", 1, &ContactPointEstimationNode::topicCallback_Twist_FT_Sensor, this);

    //     srvServer_Start_ = n_.advertiseService("start", &ContactPointEstimationNode::srvCallback_Start,
	// 			this);
    //     srvServer_Stop_ = n_.advertiseService("stop", &ContactPointEstimationNode::srvCallback_Stop, this);
	// }

	~ContactPointEstimationNode()
	{
		m_run_estimator = false;

		if (m_cpe_thread.joinable())
			m_cpe_thread.join();

		if (m_sne_thread.joinable())
			m_sne_thread.join();

		delete cpe_params_;
		delete cpe_;
		delete sne_params_;
		delete sne_;
	}


    bool loadParameters()
    {
		delete cpe_;
		delete cpe_params_;
		delete sne_;
		delete sne_params_;

		cpe_ = nullptr;
		cpe_params_ = nullptr;
		sne_ = nullptr;
		sne_params_ = nullptr;

        double gamma_r = get_parameter("gamma_r").as_double();
        double kappa_r = get_parameter("kappa_r").as_double();
        double beta_r  = get_parameter("beta_r").as_double();
		bool sim = get_parameter("sim").as_bool();
        double gamma_n = get_parameter("gamma_n").as_double();
        double beta_n  = get_parameter("beta_n").as_double();

        double cpe_freq = get_parameter("cpe_update_frequency").as_double();
        double sne_freq = get_parameter("sne_update_frequency").as_double();

        auto initial_r_vec =
            get_parameter("initial_r").as_double_array();
        auto initial_n_vec =
            get_parameter("initial_n").as_double_array();

        if (initial_r_vec.size() != 3 || initial_n_vec.size() != 3)
        {
            RCLCPP_ERROR(get_logger(), "Initial vectors must have size 3");
            return false;
        }

        Vector3d initial_r;
        Vector3d initial_n;

        for (int i = 0; i < 3; ++i)
        {
            initial_r(i) = initial_r_vec[i];
            initial_n(i) = initial_n_vec[i];
        }

        cpe_params_ = new ContactPointEstimatorParams();
        sne_params_ = new SurfaceNormalEstimatorParams();

		sim_ = sim;

		if(sim_)
		{
			 RCLCPP_INFO(get_logger(), "Running in simulation mode, subscribing to ft_sensor_sim topic");
			topicSub_FT_Sensor_Sim_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
				"/filtered_ee_wrench_wrt_sensor_frame",
				10,
				std::bind(&ContactPointEstimationNode::topicCallback_FT_Sensor_Sim, this, std::placeholders::_1));
		}
		else
		{
			 RCLCPP_INFO(get_logger(), "Running in real mode, subscribing to ft_compensated topic");
			topicSub_FT_compensated_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
				"ft_compensated", 
				10, 
				std::bind(&ContactPointEstimationNode::topicCallback_FT_compensated, this, std::placeholders::_1));
		}

		topicSub_Twist_FT_Sensor_ = this->create_subscription<geometry_msgs::msg::Twist>(
			"/cartesian_velocity_base_frame_feedback",
			10,
			std::bind(&ContactPointEstimationNode::topicCallback_Twist_FT_Sensor, this, std::placeholders::_1));

        cpe_params_->setGammaR(gamma_r);
        cpe_params_->setKappaR(kappa_r);
        cpe_params_->setBetaR(beta_r);
        cpe_params_->setInitialR(initial_r);
        cpe_params_->setUpdateFrequency(cpe_freq);

        sne_params_->setGammaN(gamma_n);
        sne_params_->setBetaN(beta_n);
        sne_params_->setInitialN(initial_n);
        sne_params_->setUpdateFrequency(sne_freq);
		// ? print gamma_n and beta_n
		RCLCPP_INFO(get_logger(), "Loaded parameters: gamma_r=%.2f, kappa_r=%.2f, beta_r=%.2f, gamma_n=%.2f, beta_n=%.2f, cpe_freq=%.2f, sne_freq=%.2f", gamma_r, kappa_r, beta_r, gamma_n, beta_n, cpe_freq, sne_freq);
        cpe_ = new ContactPointEstimator(cpe_params_);
        sne_ = new SurfaceNormalEstimator(sne_params_);

        return true;
    }

    // bool getEstimatorParameters()
	// {
	// 	// ? MODIFIED VERSION TO WORK WITH ROS2
	// 	// double gamma_r;
	// 	// if (n_.hasParam("gamma_r"))
	// 	// {
	// 	// 	n_.getParam("gamma_r", gamma_r);
	// 	// }
	// 	this->declare_parameter<double>("gamma_r");
	// 	double gamma_r = this->get_parameter("gamma_r").as_double();

	// 	else
	// 	{
	// 		ROS_ERROR("Parameter gamma_r not set, shutting down node...");
	// 		n_.shutdown();
	// 		return false;
	// 	}

	// 	// double kappa_r;
	// 	// if (n_.hasParam("kappa_r"))
	// 	// {
	// 	// 	n_.getParam("kappa_r", kappa_r);
	// 	// }
	// 	this->declare_parameter<double>("kappa_r");
	// 	double kappa_r = this->get_parameter("kappa_r").as_double();

	// 	// else
	// 	// {
	// 	// 	ROS_ERROR("Parameter kappa_r not set, shutting down node...");
	// 	// 	n_.shutdown();
	// 	// 	return false;
	// 	// }
	// 	else{
	// 		RCLCPP_ERROR(this->get_logger(), "Parameter kappa_r not set, shutting down node...");
	// 		rclcpp::shutdown();
	// 		return false;
	// 	}

	// 	double beta_r;
	// 	if (n_.hasParam("beta_r"))
	// 	{
	// 		n_.getParam("beta_r", beta_r);
	// 	}

	// 	else
	// 	{
	// 		ROS_ERROR("Parameter beta_r not set, shutting down node...");
	// 		n_.shutdown();
	// 		return false;
	// 	}


    //     /// Get initial estimate of the contact point
	// 	XmlRpc::XmlRpcValue initial_r_XmlRpc;
	// 	Vector3d initial_r;
	// 	if (n_.hasParam("initial_r"))
	// 	{
	// 		n_.getParam("initial_r", initial_r_XmlRpc);
	// 	}

	// 	else
	// 	{
	// 		ROS_ERROR("Parameter initial_r not set, shutting down node...");
	// 		n_.shutdown();
	// 		return false;
	// 	}

	// 	if(initial_r_XmlRpc.size()!=3)
	// 	{
	// 		ROS_ERROR("Wrong initial_r size.");
	// 		n_.shutdown();
	// 		return false;
	// 	}

	// 	/// Resize and assign of values to the initial_r
	// 	for (int i = 0; i < initial_r_XmlRpc.size(); i++)
	// 	{
	// 		initial_r(i) = (double)initial_r_XmlRpc[i];
	// 	}

	// 	double cpe_update_frequency;
	// 	if (n_.hasParam("cpe_update_frequency"))
	// 	{
	// 		n_.getParam("cpe_update_frequency", cpe_update_frequency);
	// 	}

	// 	else
	// 	{
	// 		ROS_ERROR("Parameter cpe_update_frequency not set, shutting down node...");
	// 		n_.shutdown();
	// 		return false;
	// 	}

	// 	double gamma_n;
	// 	if (n_.hasParam("gamma_n"))
	// 	{
	// 		n_.getParam("gamma_n", gamma_n);
	// 	}

	// 	else
	// 	{
	// 		ROS_ERROR("Parameter gamma_n not set, shutting down node...");
	// 		n_.shutdown();
	// 		return false;
	// 	}

	// 	double beta_n;
	// 	if (n_.hasParam("beta_n"))
	// 	{
	// 		n_.getParam("beta_n", beta_n);
	// 	}

	// 	else
	// 	{
	// 		ROS_ERROR("Parameter beta_n not set, shutting down node...");
	// 		n_.shutdown();
	// 		return false;
	// 	}

    //     /// Get initial estimate of the surface normal
	// 	XmlRpc::XmlRpcValue initial_n_XmlRpc;
	// 	Vector3d initial_n;
	// 	if (n_.hasParam("initial_n"))
	// 	{
	// 		n_.getParam("initial_n", initial_n_XmlRpc);
	// 	}

	// 	else
	// 	{
	// 		ROS_ERROR("Parameter initial_n not set, shutting down node...");
	// 		n_.shutdown();
	// 		return false;
	// 	}

	// 	if(initial_n_XmlRpc.size()!=3)
	// 	{
	// 		ROS_ERROR("Wrong initial_n size.");
	// 		n_.shutdown();
	// 		return false;
	// 	}

	// 	/// Resize and assign of values to the initial_n
	// 	for (int i = 0; i < initial_n_XmlRpc.size(); i++)
	// 	{
	// 		initial_n(i) = (double)initial_n_XmlRpc[i];
	// 	}


    //     double sne_update_frequency;
    //     if (n_.hasParam("sne_update_frequency"))
    //     {
    //         n_.getParam("sne_update_frequency", sne_update_frequency);
    //     }

    //     else
    //     {
    //         ROS_ERROR("Parameter sne_update_frequency not set, shutting down node...");
    //         n_.shutdown();
    //         return false;
    //     }

	// 	bool ret = true;
    //     cpe_params = new ContactPointEstimatorParams();
    //     sne_params = new SurfaceNormalEstimatorParams();

    //     cpe_params->setGammaR(gamma_r);
    //     cpe_params->setKappaR(kappa_r);
    //     cpe_params->setBetaR(beta_r);
    //     cpe_params->setInitialR(initial_r);

    //     sne_params->setGammaN(gamma_n);
    //     sne_params->setBetaN(beta_n);
    //     sne_params->setInitialN(initial_n);

    //     cpe_params->setUpdateFrequency(cpe_update_frequency);
    //     sne_params->setUpdateFrequency(sne_update_frequency);

	// 	return ret;

	// }

	// ? MODIFIED VERSION TO WORK WITH ROS2
	// void topicCallback_FT_compensated(const geometry_msgs::WrenchStampedPtr &msg)
	// {
	// 	m_ft_mutex.lock();
	// 	m_ft_compensated = *msg;
	// 	m_ft_mutex.unlock();

	// 	m_received_ft = true;
	// }

	void topicCallback_FT_Sensor_Sim(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
	{
		m_ft_mutex.lock();
		// ? Add timestamp and frame_id to the simulated wrench message
		m_ft_compensated = *msg;
		m_ft_compensated.header.stamp = this->now();
		m_ft_compensated.header.frame_id = "ur_ati_45_sensor_link";
		// m_ft_compensated.wrench = *msg;
		m_ft_mutex.unlock();

		m_received_ft = true;

		if (abs(msg ->wrench.force.z) < 0.01)
		{
			no_contact_ = true;
		}
		else
		{
			no_contact_ = false;
		}
	}

	void topicCallback_FT_compensated(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
	{
		m_ft_mutex.lock();
		m_ft_compensated = *msg;
		m_ft_mutex.unlock();

		m_received_ft = true;
	}

    // void topicCallback_Twist_FT_Sensor(const geometry_msgs::TwistStampedPtr &msg)
    // {
    // 	m_twist_mutex.lock();
    // 	m_twist_ft_sensor = *msg;
    // 	m_twist_mutex.unlock();

    //     m_received_twist = true;
    // }
	void topicCallback_Twist_FT_Sensor(const geometry_msgs::msg::Twist::SharedPtr msg)
	{
		m_twist_mutex.lock();
		// ? modified for a twist message with no header, add timestamp and frame_id
		m_twist_ft_sensor.header.stamp = this->now();
		m_twist_ft_sensor.header.frame_id = "ur_eef_tip_link";
		m_twist_ft_sensor.twist = *msg;
		// m_twist_ft_sensor = *msg;
		m_twist_mutex.unlock();

		m_received_twist = true;
	}

    // bool srvCallback_Start(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
	// {
    // 	ROS_INFO("Starting cpe + sne node");
    // 	cpe->reset();
    // 	sne->reset();

    // 	getEstimatorParameters();
    // 	m_run_estimator = true;
    // 	m_cpe_thread = boost::thread(boost::bind(&ContactPointEstimationNode::CPEThreadFunction, this));
    // 	m_sne_thread = boost::thread(boost::bind(&ContactPointEstimationNode::SNEThreadFunction, this));

	// 	return true;
	// }
	void srvCallback_Start(const std_srvs::srv::Empty::Request::SharedPtr req, std_srvs::srv::Empty::Response::SharedPtr res)
	{
		if (m_run_estimator)
		{
			RCLCPP_WARN(get_logger(), "Estimators already running");
			return;
		}
        RCLCPP_INFO(get_logger(), "Starting estimators");

        if (!loadParameters())
        {
            RCLCPP_ERROR(get_logger(), "Parameter loading failed");
            return;
        }

		cpe_->reset();
		sne_->reset();

		m_run_estimator = true;

		m_cpe_thread = boost::thread(boost::bind(&ContactPointEstimationNode::CPEThreadFunction, this));
		m_sne_thread = boost::thread(boost::bind(&ContactPointEstimationNode::SNEThreadFunction, this));

	}

    // bool srvCallback_Stop(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    // {
    // 	ROS_INFO("Stopping cpe + sne node");
//
    //     m_run_estimator = false;

    //     m_received_ft = false;
    //     m_received_twist = false;

    //     return true;
    // }
	void srvCallback_Stop(const std_srvs::srv::Empty::Request::SharedPtr req, std_srvs::srv::Empty::Response::SharedPtr res)
	{
        RCLCPP_INFO(get_logger(), "Stopping cpe + sne node");

		m_run_estimator = false;

		m_received_ft = false;
		m_received_twist = false;

	}


    bool estimatorRunning()
	{
        return m_run_estimator;
	}

    void CPEThreadFunction()
    {
    	rclcpp::Rate loop_rate(cpe_params_->getUpdateFrequency());
    	for(;;)
    	{
    		if(!m_run_estimator)
    		{
    			return;
    		}

    		else if(!m_received_ft)
    		{
    			static rclcpp::Time t = this->now();
    			if((this->now() - t).seconds() > 1.0)
    			{
    				RCLCPP_ERROR(get_logger(), "Haven't received FT sensor measurements");
    				t = this->now();
    			}

    		}

    		else if(!rclcpp::ok())
    		{
    			return;
    		}

    		else
    		{
    			m_ft_mutex.lock();
    			cpe_->update(m_ft_compensated);
    			m_ft_mutex.unlock();

    			topicPub_ContactPointEstimate_->publish(cpe_->getEstimate());
    			loop_rate.sleep();
    		}
    	}
    }

    void SNEThreadFunction()
    {
    	rclcpp::Rate loop_rate(sne_params_->getUpdateFrequency());
    	for(;;)
    	{
    		if(!m_run_estimator)
    		{
    			return;
    		}

    		else if(!m_received_twist)
    		{
				static rclcpp::Time t = this->now();
				if ((this->now() - t).seconds() > 1.0)
				{
					RCLCPP_ERROR(get_logger(), "Haven't received FT sensor measurements");
					t = this->now();
				}
    		}

    		else if(!rclcpp::ok())
    		{
    			return;
    		}

    		else
    		{
    			m_twist_mutex.lock();
				if(!no_contact_)
				{
    				sne_->update(m_twist_ft_sensor);
				}
    			m_twist_mutex.unlock();

    			topicPub_SurfaceNormalEstimate_->publish(sne_->getEstimate());
    			loop_rate.sleep();
    		}
    	}
    }




private:

    geometry_msgs::msg::WrenchStamped m_ft_compensated;
    geometry_msgs::msg::TwistStamped m_twist_ft_sensor;

    boost::mutex m_ft_mutex;
    boost::mutex m_twist_mutex;

    boost::thread m_cpe_thread;
    boost::thread m_sne_thread;

	bool m_received_ft;
    bool m_received_twist;

    bool m_run_estimator;
	bool no_contact_ = true;

	bool sim_;
    /// declaration of topics to publish
	// ? MODIFIED VERSION TO WORK WITH ROS2
	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr topicPub_ContactPointEstimate_;
	rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr topicPub_SurfaceNormalEstimate_;


    // ros::Publisher topicPub_ContactPointEstimate_;
    // ros::Publisher topicPub_SurfaceNormalEstimate_;

    /// declaration of topics to subscribe, callback is called for new messages arriving
	// ? MODIFIED VERSION TO WORK WITH ROS2
	rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr topicSub_FT_compensated_;
	// rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr topicSub_Twist_FT_Sensor_;
	// ? Modified to work with kinematic tools
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr topicSub_Twist_FT_Sensor_;

	rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr topicSub_FT_Sensor_Sim_;

    // ros::Subscriber topicSub_FT_compensated_;
    // ros::Subscriber topicSub_Twist_FT_Sensor_;

    /// declaration of service servers
	// ? MODIFIED VERSION TO WORK WITH ROS2
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srvServer_Start_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srvServer_Stop_;

    // ros::ServiceServer srvServer_Start_;
    // ros::ServiceServer srvServer_Stop_;

	// ? MODIFIED VERSION TO WORK WITH ROS2
	// ros::Time last_publish_time;
	// rclcpp::Time last_publish_time;

	ContactPointEstimatorParams *cpe_params_{nullptr};
	ContactPointEstimator *cpe_{nullptr};

	SurfaceNormalEstimatorParams *sne_params_{nullptr};
	SurfaceNormalEstimator *sne_{nullptr};
};


int main(int argc, char **argv)
{
	// ? MODIFIED VERSION TO WORK WITH ROS2
	// ros::init(argc, argv, "contact_point_estimation_node");

	// ContactPointEstimationNode cpe_node;

	// if(!cpe_node.getEstimatorParameters())
	// {
	// 	cpe_node.n_.shutdown();
	// 	return 0;
	// }


	// cpe_node.cpe = new ContactPointEstimator(cpe_node.cpe_params);
	// cpe_node.sne = new SurfaceNormalEstimator(cpe_node.sne_params);

    // ros::AsyncSpinner s(4);
    // s.start();

    // ros::waitForShutdown();

	// return 0;
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ContactPointEstimationNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}


