#ifndef SLG_MSGS__SEGMENT2D_HPP_
#define SLG_MSGS__SEGMENT2D_HPP_
#include "rclcpp/rclcpp.hpp" 
#include <vector>
#include <algorithm>
#include <numeric>

#include <slg_msgs/msg/segment.hpp>

#include "point2D.hpp"

namespace slg{
class Segment2D{
	public:
		Segment2D() : id(0), 
					  label(BACKGROUND), 
					  angular_distance_to_closest_boundary(0.0), 
					  last_point_prior_seg(Point2D(0, 0)), 
					  first_point_next_seg(Point2D(0, 0)), 
					  last_centroid(Point2D(0, 0)) {}

		Segment2D(int newId, Point2D prevPoint, Point2D currPoint, Point2D nextPoint) : 
					id(newId), 
					label(BACKGROUND), 
					angular_distance_to_closest_boundary(0.0), 
					points({currPoint}), 
					last_point_prior_seg(prevPoint), 
					first_point_next_seg(nextPoint), 
					last_centroid(currPoint) {}

		Segment2D(const Segment2D& seg) : 
					id(seg.get_id()), 
					label(seg.get_label()), 
					angular_distance_to_closest_boundary(0.0), 
					points(seg.get_points()), 
					last_point_prior_seg(seg.get_prior_segment()), 
					first_point_next_seg(seg.get_next_segment()), 
					last_centroid(seg.get_last_centroid()) {}

		Segment2D(const slg_msgs::msg::Segment& segmentMsg) : 
					id(segmentMsg.id), 
					label(slg::Label(segmentMsg.label)), 
					angular_distance_to_closest_boundary(segmentMsg.angular_distance),
					last_point_prior_seg(segmentMsg.last_point_prior_segment), 
					first_point_next_seg(segmentMsg.first_point_next_segment) 
					{
						points.insert(points.begin(), std::begin(segmentMsg.points), std::end(segmentMsg.points));
					}

		~Segment2D(){}

		operator slg_msgs::msg::Segment() const{
			slg_msgs::msg::Segment segmentMsg;

			// Transform the segment in message
			segmentMsg.id = id;
			segmentMsg.label = label;
			segmentMsg.angular_distance = angular_distance_to_closest_boundary;
			segmentMsg.last_point_prior_segment.x = last_point_prior_seg.x;
			segmentMsg.last_point_prior_segment.y = last_point_prior_seg.y;
			segmentMsg.first_point_next_segment.x = first_point_next_seg.x;
			segmentMsg.first_point_next_segment.y = first_point_next_seg.y;

			for (const auto& point : points){
				segmentMsg.points.push_back(point);
			}

			return segmentMsg;
		}

		Segment2D& operator= (const Segment2D & seg){
			if (this != &seg){
				this->id = seg.get_id();
				this->label = seg.get_label();
				this->points = seg.get_points();
				this->last_point_prior_seg = seg.get_prior_segment();
				this->first_point_next_seg = seg.get_next_segment();
				this->last_centroid = seg.get_last_centroid();
			}
			return *this;
		}

		Segment2D& operator= (const slg_msgs::msg::Segment & segmentMsg){
			*this = Segment2D(segmentMsg);
			return *this;
		}

		int size() 				const		{ return points.size(); }
		bool empty() 			const		{ return points.empty(); }
		void clear() 						{ points.clear(); id = 0; label = BACKGROUND; }
		double width() 			const		{ return (points.back() - points.front()).length(); }
		double width_squared() 	const		{ return (points.back() - points.front()).length_squared(); }
		Point2D first_point()	const		{ return points.front(); }
		Point2D last_point()	const		{ return points.back(); }
		Point2D vector()		const		{ return points.back() - points.front(); }
		double min_angle()		const		{ return points.front().angle(); }
		double max_angle()		const		{ return points.back().angle(); }
		double mean_angle()		const		{ return (points.front().angle() + points.back().angle()) / 2.0; }
		int get_id()			const		{ return id; }
		Label get_label()		const		{ return label; }
		std::vector<Point2D> get_points() const{ return points; }
		Point2D get_prior_segment()	const		{ return last_point_prior_seg; }
		Point2D get_next_segment()	const		{ return first_point_next_seg; }
		Point2D get_last_centroid()	const		{ return last_centroid; }
		double get_angular_distance_to_closest_boundary() const {return angular_distance_to_closest_boundary; }
		void set_id(int id)					{ this->id = id; }
		void set_label(Label label)			{ this->label = label; }
		void set_prior_segment(Point2D point)	{ last_point_prior_seg = point; }
		void set_next_segment(Point2D point)	{ first_point_next_seg = point; }
		void set_last_centroid(Point2D point)	{ last_centroid = point; }
		void set_angular_distance_to_closest_boundary(double angle) {angular_distance_to_closest_boundary = angle; }

		double orientation(){
			return (vector().y != 0.0) ? Point2D(-1, - vector().x / vector().y).angle() : 0.0;
		}

		Point2D projection(const Point2D& p) const {
			Point2D a = points.back() - points.front();
			Point2D b = p - points.front();
			return points.front() + a.dot(b) * a / a.length_squared();
		}

		double distance_to(const Point2D& p) const {
			return (p - projection(p)).length();
		}

		Point2D centroid() const{
			Point2D sum = std::accumulate(points.begin(), points.end(), Point2D(0.0, 0.0));
			return sum / points.size();
		}

        Point2D minimumPoint() const {
            // Check if the points vector is not empty
            if (points.empty()) {
                // Return a default Point2D or handle the case as needed
                return Point2D(0.0, 0.0);
            }

            // Find the minimum x and y values using std::min_element
            double min_x = std::min_element(points.begin(), points.end(), [](const Point2D& a, const Point2D& b) {
                return a.x < b.x;
            })->x;

            double min_y = std::min_element(points.begin(), points.end(), [](const Point2D& a, const Point2D& b) {
                return a.y < b.y;
            })->y;

            // Return the Point2D with minimum x and y
            return Point2D(min_x, min_y);
        }

        Point2D maximumPoint() const {
            // Check if the points vector is not empty
            if (points.empty()) {
                // Return a default Point2D or handle the case as needed
                return Point2D(0.0, 0.0);
            }

            // Find the maximum x and y values using std::max_element
            double max_x = std::max_element(points.begin(), points.end(), [](const Point2D& a, const Point2D& b) {
                return a.x < b.x;
            })->x;

            double max_y = std::max_element(points.begin(), points.end(), [](const Point2D& a, const Point2D& b) {
                return a.y < b.y;
            })->y;

            // Return the Point2D with maximum x and y
            return Point2D(max_x, max_y);
        }

        Point2D nearestPoint() const {
            // Check if the points vector is not empty
            if (points.empty()) {
                // Return a default Point2D or handle the case as needed
                return Point2D(0.0, 0.0);
            }

            // Find the point with the minimum distance to the origin (0, 0)
            auto nearestIter = std::min_element(points.begin(), points.end(), [](const Point2D& a, const Point2D& b) {
                return a.length() < b.length();
            });

            return *nearestIter;
        }


		double height() const{
			auto maxPoint = std::max_element(points.begin(), points.end(), [](Point2D p1, Point2D p2){return p1.y < p2.y;});
			return distance_to(*maxPoint);
		}

		void add_point(Point2D point){
			points.push_back(point);
			last_centroid = centroid();
		}

		void add_points(std::vector<Point2D> newPoints){
			points.insert(points.end(), newPoints.begin(), newPoints.end());
		}

		void merge(Segment2D seg){
			if (label != seg.get_label()) label = BACKGROUND;

			std::vector<Point2D> newPoints = seg.get_points();
			points.insert(points.end(), newPoints.begin(), newPoints.end());
			first_point_next_seg = seg.get_next_segment();
			last_centroid = centroid();
		}

		Segment2D left_split(int index) {
            std::vector<Point2D> left(points.begin(), points.begin() + index);

            Segment2D splited_segment;
            splited_segment.add_points(left);
            splited_segment.set_id(this->id);
            splited_segment.set_label(this->label);
            splited_segment.set_prior_segment(this->last_point_prior_seg);
            splited_segment.set_next_segment(points[index + 1]);

            return splited_segment;
        }

        // Non-const version of right_split
        Segment2D right_split(int index) {
            std::vector<Point2D> right(points.begin() + index, points.end());

            Segment2D splited_segment;
            splited_segment.add_points(right);
            splited_segment.set_id(this->id);
            splited_segment.set_label(this->label);
            splited_segment.set_prior_segment(points[index - 1]);
            splited_segment.set_next_segment(this->first_point_next_seg);

            return splited_segment;
        }

        bool IsLShaped() const {
            std::vector<Point2D> checking_points(points.begin(), points.end());

            double vector1x = checking_points[5].x - checking_points.front().x;
            double vector1y = checking_points[5].y - checking_points.front().y;

            double vector2x = checking_points[checking_points.size() - 7].x - checking_points.back().x;
            double vector2y = checking_points[checking_points.size() - 7].y - checking_points.back().y;

            // RCLCPP_INFO(rclcpp::get_logger("segment2d"), "checking points 01 x: (%f, %f)", checking_points.front().x, checking_points[1].x);
            // RCLCPP_INFO(rclcpp::get_logger("segment2d"), "checking points 01 y: (%f, %f)", checking_points.front().y, checking_points[1].y);
            // RCLCPP_INFO(rclcpp::get_logger("segment2d"), "checking points -2-1 x: (%f, %f)", checking_points[checking_points.size() - 2].x, checking_points.back().x);
            // RCLCPP_INFO(rclcpp::get_logger("segment2d"), "checking points -2-1 y: (%f, %f)", checking_points[checking_points.size() - 2].y, checking_points.back().y);
            // // Print the values for debugging
            // RCLCPP_INFO(rclcpp::get_logger("segment2d"), "Vector1: (%f, %f)", vector1x, vector1y);
            // RCLCPP_INFO(rclcpp::get_logger("segment2d"), "Vector2: (%f, %f)", vector2x, vector2y);

            // Check for a right angle using the dot product
            double dotProduct = vector1x * vector2x + vector1y * vector2y;
            double magnitudeProduct = sqrt(pow(vector1x, 2) + pow(vector1y, 2)) *
                                    sqrt(pow(vector2x, 2) + pow(vector2y, 2));
            double cosTheta = dotProduct / magnitudeProduct;

            // Print the cosine value for debugging
            // RCLCPP_INFO(rclcpp::get_logger("segment2d"), "Cosine Theta: %f", cosTheta);

            // You can adjust this threshold based on your specific requirements
            return std::abs(cosTheta) > 0.4;
        }

    public:
		int id;
	private:
		Label label;
		double angular_distance_to_closest_boundary;
		std::vector<Point2D> points;
		Point2D last_point_prior_seg, first_point_next_seg, last_centroid;
};
}  // namespace slg

#endif  // SLG_MSGS__SEGMENT2D_HPP_
