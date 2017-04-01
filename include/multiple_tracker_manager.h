#include <map>
#include <memory>
#include "KalmanFilter.hpp"
#include "Object.hpp"
#include "Detection.hpp"
#include "Tracker.hpp"
#include "HungarianAssociator.hpp"
#include "Comparator.hpp"
#include <ros/ros.h>

class Robot : public Object
{
	public:
		Robot(const VectorXd & state_, const MatrixXd & covariance_) : state(state_), covariance(covariance_)
		{
			objectType=Object::TYPE_CYLINDER;
		};

		~Robot()
		{};

		void setState(const VectorXd & state_)
		{
			this->state=state_;
		}

		void setCovariance(const MatrixXd & covariance_)
		{
			this->covariance=covariance_;

		}

		double compareWith(Object & otherObject, int mode, int metric)
		{
			assert(this->getObjectType() == otherObject.getObjectType() && "Objects should have the same type if you wish to compare them. Check the types of detections and tracks if you are using them.");

			Robot &otherPose = dynamic_cast<Robot&>(otherObject);

			switch (mode)
			{
				case(COMP_POSITION):

					assert((metric == Comparator::METRIC_MAHALANOBIS || metric == Comparator::METRIC_EUCLIDEAN) && "Position distance can only use the Euclidean or the Mahalanobis metrics for now.");

					//Mahalanobis distance between the detected person's position and this ones
					//position distribution

					if (metric == Comparator::METRIC_MAHALANOBIS)
					{

						return Comparator::mahalanobis(otherPose.state, this->state, this->covariance.block(0,0,otherPose.state.rows(),otherPose.state.rows()));
					}

					else if (metric == Comparator::METRIC_EUCLIDEAN)
					{
						return Comparator::euclidean(otherPose.state, this->state);
					}

					break;
				default:

					break;
			}

			return 0.0;
		}

		VectorXd getObservableStates()	
		{
			return this->state;
		}

		MatrixXd getObervableCovariance()
		{
			return this->covariance;
		}

		std::shared_ptr<Object> clone()
		{
			return std::shared_ptr<Object>(new Robot(this->state, this->covariance));
		}


	protected:
		// Attributes
		VectorXd state;
		MatrixXd covariance;
};

class Cylinder : public Object
{
	public:
		Cylinder(const VectorXd & state_, const MatrixXd & covariance_) : state(state_), covariance(covariance_)
		{
			objectType=Object::TYPE_CYLINDER;
		};

		~Cylinder()
		{};

		void setState(const VectorXd & state_)
		{
			this->state=state_;
		}

		void setCovariance(const MatrixXd & covariance_)
		{
			this->covariance=covariance_;

		}

		double compareWith(Object & otherObject, int mode, int metric)
		{
			assert(this->getObjectType() == otherObject.getObjectType() && "Objects should have the same type if you wish to compare them. Check the types of detections and tracks if you are using them.");

			Cylinder &otherPerson = dynamic_cast<Cylinder&>(otherObject);

			switch (mode)
			{
				case(COMP_POSITION):

					assert((metric == Comparator::METRIC_MAHALANOBIS || metric == Comparator::METRIC_EUCLIDEAN) && "Position distance can only use the Euclidean or the Mahalanobis metrics for now.");

					//Mahalanobis distance between the detected person's position and this ones
					//position distribution

					if (metric == Comparator::METRIC_MAHALANOBIS)
					{

						return Comparator::mahalanobis(otherPerson.state, this->state, this->covariance.block(0,0,otherPerson.state.rows(),otherPerson.state.rows()));
					}

					else if (metric == Comparator::METRIC_EUCLIDEAN)
					{
						return Comparator::euclidean(otherPerson.state, this->state);
					}

					break;
				default:

					break;
			}

			return 0.0;
		}

		VectorXd getObservableStates()	
		{
			return this->state;
		}

		MatrixXd getObervableCovariance()
		{
			return this->covariance;
		}

		std::shared_ptr<Object> clone()
		{
			return std::shared_ptr<Object>(new Cylinder(this->state, this->covariance));
		}


	protected:
		// Attributes
		VectorXd state;
		MatrixXd covariance;
};

class MultipleTrackerManager
{
	public:
	int current_index;
	std::vector<std::shared_ptr<Tracker<Cylinder, KalmanFilter> > > trackers;
	std::shared_ptr<Tracker<Robot,KalmanFilter> > pose; // Robot pose
	protected:

	std::shared_ptr<KalmanFilter> trackerInit(const Eigen::VectorXd & initial_state, const Eigen::MatrixXd & initial_cov)
	{
		/* CYLINDER HAS 8 PARAMETERS (px,py,pz,dx,dy,dz,r,h) */
		MatrixXd stateTransitionModel(14,14);

		stateTransitionModel << 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
					0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
					0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
					0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
					0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0,
					0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1,
					0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
					0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
					0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
					0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
					0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
					0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
					0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
					0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

		MatrixXd observationModel(8,14);
		observationModel << 	1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
					0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
					0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
					0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
					0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
					0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
					0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
					0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;

		double T = 0.03;

		double vel_scale=10000.0;
		MatrixXd processNoiseCovariance(14,14);
		processNoiseCovariance << pow(T,4)/4, 0, 0, 0, 0, 0, 0, 0, vel_scale*pow(T, 3)/2, 0, 0, 0, 0, 0,
        					0, pow(T,4)/4, 0, 0, 0, 0, 0, 0, 0, vel_scale*pow(T, 3)/2, 0, 0, 0, 0,
						0, 0, pow(T,4)/4, 0, 0, 0, 0, 0, 0, 0, vel_scale*pow(T, 3)/2, 0, 0, 0, 
						0, 0, 0, pow(T,4)/4, 0, 0, 0, 0, 0, 0, 0, vel_scale*pow(T, 3)/2, 0, 0,
						0, 0, 0, 0, pow(T,4)/4, 0, 0, 0, 0, 0, 0, 0, vel_scale*pow(T, 3)/2, 0,
						0, 0, 0, 0, 0, pow(T,4)/4, 0, 0, 0, 0, 0, 0, 0, vel_scale*pow(T, 3)/2,
						0, 0, 0, 0, 0, 0,  pow(T, 2), 0, 0, 0, 0, 0, 0, 0,
						0, 0, 0, 0, 0, 0, 0,  pow(T, 2), 0, 0, 0, 0, 0, 0,
						vel_scale*pow(T, 3)/2, 0, 0, 0, 0, 0, 0, 0,  pow(T, 2), 0, 0, 0, 0, 0,
						0, vel_scale*pow(T, 3)/2, 0, 0, 0, 0, 0, 0, 0,  pow(T, 2), 0, 0, 0, 0,
						0, 0, vel_scale*pow(T, 3)/2, 0, 0, 0, 0, 0, 0, 0,  pow(T, 2), 0, 0, 0,
						0, 0, 0, vel_scale*pow(T, 3)/2, 0, 0, 0, 0, 0, 0, 0,  pow(T, 2), 0, 0,
						0, 0, 0, 0, vel_scale*pow(T, 3)/2, 0, 0, 0, 0, 0, 0, 0,  pow(T, 2), 0,
						0, 0, 0, 0, 0, vel_scale*pow(T, 3)/2, 0, 0, 0, 0, 0, 0, 0,  pow(T, 2);

		processNoiseCovariance=processNoiseCovariance*1.0;

		MatrixXd observationNoiseCov(8,8);
		observationNoiseCov << 1, 0, 0, 0, 0, 0, 0, 0,
				       0, 1, 0, 0, 0, 0, 0, 0,
				       0, 0, 1, 0, 0, 0, 0, 0,
				       0, 0, 0, 1, 0, 0, 0, 0,
				       0, 0, 0, 0, 1, 0, 0, 0,
				       0, 0, 0, 0, 0, 1, 0, 0,
				       0, 0, 0, 0, 0, 0, 1, 0,
				       0, 0, 0, 0, 0, 0, 0, 1;

		observationNoiseCov=observationNoiseCov*10000.0;



		MatrixXd controlModel(14,6);

		controlModel << 1, 0, 0, 0, 0, 0,
				0, 1, 0, 0, 0, 0, 
				0, 0, 1, 0, 0, 0,
				0, 0, 0, 1, 0, 0,
				0, 0, 0, 0, 1, 0,
				0, 0, 0, 0, 0, 1, 
				0, 0, 0, 0, 0, 0, 
				0, 0, 0, 0, 0, 0, 
				0, 0, 0, 0, 0, 0, 
				0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 
				0, 0, 0, 0, 0, 0, 
				0, 0, 0, 0, 0, 0, 
				0, 0, 0, 0, 0, 0;


		return std::shared_ptr<KalmanFilter>(new KalmanFilter(stateTransitionModel, controlModel, observationModel, processNoiseCovariance, observationNoiseCov, initial_state, initial_cov));
	}


	std::shared_ptr<KalmanFilter> poseTrackerInit(const Eigen::VectorXd & initial_state, const Eigen::MatrixXd & initial_cov)
	{
		/* CYLINDER HAS 8 PARAMETERS (px,py,pz,dx,dy,dz,r,h) */
		MatrixXd stateTransitionModel(6,6);

		stateTransitionModel << 1, 0, 0, 0, 0, 0,
					0, 1, 0, 0, 0, 0,
					0, 0, 1, 0, 0, 0,
					0, 0, 0, 1, 0, 0,
					0, 0, 0, 0, 1, 0,
					0, 0, 0, 0, 0, 1;


		MatrixXd observationModel(6,6);
		observationModel <<	1, 0, 0, 0, 0, 0,
					0, 1, 0, 0, 0, 0,
					0, 0, 1, 0, 0, 0,
					0, 0, 0, 1, 0, 0,
					0, 0, 0, 0, 1, 0,
					0, 0, 0, 0, 0, 1;

		double T = 0.03;

		double vel_scale=10000.0;
		MatrixXd processNoiseCovariance(12,12);
		processNoiseCovariance << 	pow(T,4)/4, 0, 0, 0, 0, 0, vel_scale*pow(T, 3)/2, 0, 0, 0, 0, 0,
        					0, pow(T,4)/4, 0, 0, 0, 0, 0, vel_scale*pow(T, 3)/2, 0, 0, 0, 0,
						0, 0, pow(T,4)/4, 0, 0, 0, 0, 0, vel_scale*pow(T, 3)/2, 0, 0, 0, 
						0, 0, 0, pow(T,4)/4, 0, 0, 0, 0, 0, vel_scale*pow(T, 3)/2, 0, 0,
						0, 0, 0, 0, pow(T,4)/4, 0, 0, 0, 0, 0, vel_scale*pow(T, 3)/2, 0,
						0, 0, 0, 0, 0, pow(T,4)/4, 0, 0, 0, 0, 0, vel_scale*pow(T, 3)/2,
						vel_scale*pow(T, 3)/2, 0, 0, 0, 0, 0, pow(T, 2), 0, 0, 0, 0, 0,
						0, vel_scale*pow(T, 3)/2, 0, 0, 0, 0, 0, pow(T, 2), 0, 0, 0, 0,
						0, 0, vel_scale*pow(T, 3)/2, 0, 0, 0, 0, 0, pow(T, 2), 0, 0, 0,
						0, 0, 0, vel_scale*pow(T, 3)/2, 0, 0, 0, 0, 0, pow(T, 2), 0, 0,
						0, 0, 0, 0, vel_scale*pow(T, 3)/2, 0, 0, 0, 0, 0, pow(T, 2), 0,
						0, 0, 0, 0, 0, vel_scale*pow(T, 3)/2, 0, 0, 0, 0, 0,  pow(T, 2);

		processNoiseCovariance=processNoiseCovariance*1.0;

		MatrixXd observationNoiseCov(6,6);
		observationNoiseCov << 1, 0, 0, 0, 0, 0,
				       0, 1, 0, 0, 0, 0,
				       0, 0, 1, 0, 0, 0,
				       0, 0, 0, 1, 0, 0,
				       0, 0, 0, 0, 1, 0,
				       0, 0, 0, 0, 0, 1;


		observationNoiseCov=observationNoiseCov*10000.0;

		MatrixXd controlModel(6,6);

		controlModel << 1, 0, 0, 0, 0, 0,
				0, 1, 0, 0, 0, 0, 
				0, 0, 1, 0, 0, 0,
				0, 0, 0, 1, 0, 0,
				0, 0, 0, 0, 1, 0,
				0, 0, 0, 0, 0, 1;

		return std::shared_ptr<KalmanFilter>(new KalmanFilter(stateTransitionModel, controlModel, observationModel, processNoiseCovariance, observationNoiseCov, initial_state, initial_cov));
	}


	public:

	MultipleTrackerManager() :current_index(0) {
		
	};

	// Init robot pose
	void initFilter(const Eigen::Matrix<double,6,1> & mu_, const Eigen::Matrix<double,6,6> & cov_)
	{

		std::shared_ptr<Robot> init_state(new Robot(mu_, cov_));
		pose=std::shared_ptr<Tracker<Robot,KalmanFilter> > (new Tracker<Robot,KalmanFilter>(init_state,poseTrackerInit(mu_,cov_)));
	}

	// Update robot pose
	void updateFilter(const Eigen::Matrix<double,6,1> & mu_, const Eigen::Matrix<double,6,6> & cov_)
	{
			// Convert to right struct
			std::shared_ptr<Robot> robot_pose(new Robot(mu_, cov_));
			std::shared_ptr<Detection<Robot> > detection(new Detection<Robot>(robot_pose, "Camera"));
			
			//std::shared_ptr<Tracker<Robot,KalmanFilter> > aux(new Tracker<Robot,KalmanFilter>(unDetections[a]->getObjPTR(),trackerInit(initial_state,initial_cov)));
	}

	// predict using odometry
	void predict(const Eigen::Matrix4d & transf, const Eigen::Matrix<double,6,6> & cov_)
	{
		//VectorXd temp(6);
		//temp=mu_;
		// 0. Predict
		for(unsigned int t=0; t<trackers.size();++t)
		{

			VectorXd temp(6);
			temp << 0, 0, 0, 0, 0, 0;


			// Apply transform to position
			temp.head(3) =transf.block(0,3,3,1) - trackers[t]->getObjPTR()->getObservableStates().segment(0,3);
			temp.head(3)+=transf.block(0,0,3,3) * trackers[t]->getObjPTR()->getObservableStates().segment(0,3);

			// Direction part
			//temp.segment(3,3)=transf.block(0,0,3,3).inverse()*trackers[t]->getObjPTR()->getObservableStates().segment(3,3);

               		//it_mmae->statePost(cv::Range(0,2),cv::Range(0,1)) = odom_trans + odom_rot*it_mmae->statePost(cv::Range(0,2),cv::Range(0,1));
			//ROS_ERROR_STREAM("TEMP:"<< temp);
			//VectorXd state_(6);
			//state_.head(6)=unDetections[a]->getObjPTR()->getObservableStates().head(6);

			// Apply transform to position
			//temp.head(3)=-trackers[t]->getObjPTR()->getObservableStates().segment(0,3);
			//temp.head(3)+=(relative_motion.block(0,3,3,1)+(relative_motion.block(0,0,3,3)*trackers[t]->getObjPTR()->getObservableStates().segment(0,3)));

			//temp.segment(3,3)=relative_motion.block(0,0,3,3)*trackers[t]->getObjPTR()->getObservableStates().segment(3,3);

			//VectorXd temp=VectorXd();

               		//it_mmae->statePost(cv::Range(0,2),cv::Range(0,1)) = odom_trans + odom_rot*it_mmae->statePost(cv::Range(0,2),cv::Range(0,1));
			trackers[t]->predict(temp);
		}

	}

	// Process new measurements
	const std::vector<std::shared_ptr<Tracker<Cylinder, KalmanFilter> > > & update(std::vector<Eigen::VectorXd> & detections_, const Eigen::Matrix4d & relative_motion)
	{

		// KALMAN INIT COV
		MatrixXd initial_cov(14,14);
		initial_cov <<  1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			        0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			        0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			        0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			        0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			        0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
			        0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
		initial_cov=initial_cov*1000.0;

		// DETECTIONS COV
		MatrixXd initial_dummy_det_cov(8,8);
		initial_dummy_det_cov <<  	1, 0, 0, 0, 0, 0, 0, 0, 
						0, 1, 0, 0, 0, 0, 0, 0,
						0, 0, 1, 0, 0, 0, 0, 0,  
						0, 0, 0, 1, 0, 0, 0, 0,
						0, 0, 0, 0, 1, 0, 0, 0,
						0, 0, 0, 0, 0, 1, 0, 0,
						0, 0, 0, 0, 0, 0, 1, 0, 
						0, 0, 0, 0, 0, 0, 0, 1;


		initial_dummy_det_cov=initial_dummy_det_cov*10000000000000000000.0;
		std::vector<std::shared_ptr<Detection<Cylinder>>> detectionList;

		// 0. Predict
		for(unsigned int t=0; t<trackers.size();++t)
		{
			VectorXd temp(6);
			temp << 0, 0, 0, 0, 0, 0;
			//temp.head(3) = relative_motion.block(0,3,3,1);
               		//it_mmae->statePost(cv::Range(0,2),cv::Range(0,1)) = odom_trans + odom_rot*it_mmae->statePost(cv::Range(0,2),cv::Range(0,1));
			//ROS_ERROR_STREAM("TEMP:"<< temp);
			//VectorXd state_(6);
			//state_.head(6)=unDetections[a]->getObjPTR()->getObservableStates().head(6);

			// Apply transform to position
			//temp.head(3)=-trackers[t]->getObjPTR()->getObservableStates().segment(0,3);
			//temp.head(3)+=(relative_motion.block(0,3,3,1)+(relative_motion.block(0,0,3,3)*trackers[t]->getObjPTR()->getObservableStates().segment(0,3)));

			//temp.segment(3,3)=relative_motion.block(0,0,3,3)*trackers[t]->getObjPTR()->getObservableStates().segment(3,3);

			//VectorXd temp=VectorXd();
			trackers[t]->predict(temp);

		}


		// 1. Transform detections to right structure
		for(unsigned int d=0; d<detections_.size(); ++d)
		{
			std::shared_ptr<Cylinder> cyl(new Cylinder(detections_[d], initial_dummy_det_cov));
			std::shared_ptr<Detection<Cylinder>> detection(new Detection<Cylinder>(cyl, "Camera"));
			detectionList.push_back(detection);
		}
	


		// 2. Associate
		HungarianAssociator<Cylinder, Tracker<Cylinder, KalmanFilter> > associator(Cylinder::COMP_POSITION, Comparator::METRIC_MAHALANOBIS);

		AssociationList<Cylinder, Tracker<Cylinder, KalmanFilter> > assocList = associator.associateData(trackers, detectionList);

		
		vector<Association<Cylinder, Tracker<Cylinder, KalmanFilter>>> success = assocList.getSuccessfulAssociations();
		vector<shared_ptr<Detection<Cylinder> > > unDetections = assocList.getUnassociatedDetections();

		// 3. Update associated trackers
		for(unsigned int a=0; a<success.size();++a)
		{
			success[a].getTrackerPTR()->update(*(success[a].getDetectionPTR()));
		}

		// 4. Create trackers for unassociated detections
		for(unsigned int a=0; a<unDetections.size();++a)
		{
			Eigen::VectorXd initial_state(14);
			initial_state=Eigen::VectorXd::Zero(14);

			initial_state.head(8)=unDetections[a]->getObjPTR()->getObservableStates().head(8);
			std::shared_ptr<Tracker<Cylinder,KalmanFilter> > aux(new Tracker<Cylinder,KalmanFilter>(unDetections[a]->getObjPTR(),trackerInit(initial_state,initial_cov)));
			aux->setTrackerId(current_index);
			trackers.push_back(aux);
			current_index++;
		}

		return trackers;
	}
};

