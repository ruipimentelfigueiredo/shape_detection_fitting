#include <map>
#include <memory>
#include "KalmanFilter.hpp"
#include "Object.hpp"
#include "Detection.hpp"
#include "Tracker.hpp"
#include "HungarianAssociator.hpp"
#include "Comparator.hpp"
#include <ros/ros.h>
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

	public:

	MultipleTrackerManager() :current_index(0) {
		
	};


	// Process new measurements
	const std::vector<std::shared_ptr<Tracker<Cylinder, KalmanFilter> > > & process(std::vector<Eigen::VectorXd> & detections_)
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

