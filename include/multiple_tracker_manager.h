#include <map>
#include <memory>
#include "KalmanFilter.hpp"
#include "Object.hpp"
#include "Detection.hpp"
#include "Tracker.hpp"
#include "HungarianAssociator.hpp"
#include "Comparator.hpp"

class Cylinder : public Object
{
	public:
		Cylinder(const VectorXd & state_, const MatrixXd & covariance_) : state(state_), covariance(covariance_)
		{};

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
			
		}

		VectorXd getState()	
		{
			return this->state;
		}

		MatrixXd getCovariance()
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
	static int current_index;
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

		double T = 0.5;

		MatrixXd processNoiseCovariance(14,14);
		processNoiseCovariance << pow(T,4)/4, 0, 0, 0, 0, 0, 0, 0, pow(T, 3)/2, 0, 0, 0, 0, 0,
        					0, pow(T,4)/4, 0, 0, 0, 0, 0, 0, 0, pow(T, 3)/2, 0, 0, 0, 0,
						0, 0, pow(T,4)/4, 0, 0, 0, 0, 0, 0, 0, pow(T, 3)/2, 0, 0, 0, 
						0, 0, 0, pow(T,4)/4, 0, 0, 0, 0, 0, 0, 0, pow(T, 3)/2, 0, 0,
						0, 0, 0, 0, pow(T,4)/4, 0, 0, 0, 0, 0, 0, 0, pow(T, 3)/2, 0,
						0, 0, 0, 0, 0, pow(T,4)/4, 0, 0, 0, 0, 0, 0, 0, pow(T, 3)/2,
						0, 0, 0, 0, 0, 0,  pow(T, 2), 0, 0, 0, 0, 0, 0, 0,
						0, 0, 0, 0, 0, 0, 0,  pow(T, 2), 0, 0, 0, 0, 0, 0,
						0, 0, 0, 0, 0, 0, 0, 0,  pow(T, 2), 0, 0, 0, 0, 0,
						0, 0, 0, 0, 0, 0, 0, 0, 0,  pow(T, 2), 0, 0, 0, 0,
						0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  pow(T, 2), 0, 0, 0,
						0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  pow(T, 2), 0, 0,
						0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  pow(T, 2), 0,
						0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  pow(T, 2);

		MatrixXd observationNoiseCov(8,8);
		observationNoiseCov << 1, 0, 0, 0, 0, 0, 0, 0,
				       0, 1, 0, 0, 0, 0, 0, 0,
				       0, 0, 1, 0, 0, 0, 0, 0,
				       0, 0, 0, 1, 0, 0, 0, 0,
				       0, 0, 0, 0, 1, 0, 0, 0,
				       0, 0, 0, 0, 0, 1, 0, 0,
				       0, 0, 0, 0, 0, 0, 1, 0,
				       0, 0, 0, 0, 0, 0, 0, 1;

		
		return std::shared_ptr<KalmanFilter>(new KalmanFilter(stateTransitionModel, observationModel, processNoiseCovariance, observationNoiseCov, initial_state, initial_cov));
	}

	public:

	MultipleTrackerManager() {};


	// Process new measurements
	void process(std::vector<Eigen::VectorXd> & detections_)
	{
		MatrixXd initial_cov(8,8);
		initial_cov << 1, 0, 0, 0, 0, 0, 0, 0,
			       0, 1, 0, 0, 0, 0, 0, 0,
			       0, 0, 1, 0, 0, 0, 0, 0,
			       0, 0, 0, 1, 0, 0, 0, 0,
			       0, 0, 0, 0, 1, 0, 0, 0,
			       0, 0, 0, 0, 0, 1, 0, 0,
			       0, 0, 0, 0, 0, 0, 1, 0,
			       0, 0, 0, 0, 0, 0, 0, 1;
		std::vector<std::shared_ptr<Detection<Cylinder>>> detectionList;
		// 1. Transform detections to right structure
		for(unsigned int d=0; d<detections_.size(); ++d)
		{
			std::shared_ptr<Cylinder> cyl(new Cylinder(detections_[d], initial_cov));
			std::shared_ptr<Detection<Cylinder>> detection1(new Detection<Cylinder>(cyl, "Camera"));
			detectionList.push_back(detection1);
		}

		// 2. Associate
		HungarianAssociator<Cylinder, Tracker<Cylinder, KalmanFilter> > associator(Cylinder::COMP_POSITION, Comparator::METRIC_EUCLIDEAN);
		AssociationList<Cylinder, Tracker<Cylinder, KalmanFilter> > assocList = associator.associateData(trackers, detectionList);
	
		vector<Association<Cylinder, Tracker<Cylinder, KalmanFilter>>> success = assocList.getSuccessfulAssociations();
		vector<shared_ptr<Detection<Cylinder> > > unDetections = assocList.getUnassociatedDetections();

		// 3. Update associated trackers
		for(unsigned int a=0; a<success.size();++a)
		{
			success[a].getTrackerPTR()->update(*(success[a].getDetectionPTR()));
		}

		// 4. Create trackers for unassociated detections
		for(unsigned int a=0; a<success.size();++a)
		{
			std::shared_ptr<Tracker<Cylinder,KalmanFilter> > aux(new Tracker<Cylinder,KalmanFilter>(unDetections[a]->getObjPTR(),trackerInit(unDetections[a]->getObjPTR()->getState(),unDetections[a]->getObjPTR()->getCovariance())));
			trackers.push_back(aux);
		}
	}
};

