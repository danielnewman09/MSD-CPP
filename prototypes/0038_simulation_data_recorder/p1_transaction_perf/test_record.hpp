#ifndef TEST_RECORD_HPP
#define TEST_RECORD_HPP

#include <cpp_sqlite/src/cpp_sqlite/DBBaseTransferObject.hpp>
#include <boost/describe.hpp>

/**
 * @brief Test record simulating InertialStateRecord structure
 *
 * Mimics the approximate size and field count of InertialStateRecord
 * for realistic performance testing.
 */
struct TestRecord : public cpp_sqlite::BaseTransferObject {
    using Id = cpp_sqlite::PrimaryKey<TestRecord>;

    double simulation_time{std::numeric_limits<double>::quiet_NaN()};
    double position_x{std::numeric_limits<double>::quiet_NaN()};
    double position_y{std::numeric_limits<double>::quiet_NaN()};
    double position_z{std::numeric_limits<double>::quiet_NaN()};
    double velocity_x{std::numeric_limits<double>::quiet_NaN()};
    double velocity_y{std::numeric_limits<double>::quiet_NaN()};
    double velocity_z{std::numeric_limits<double>::quiet_NaN()};
    double orientation_w{std::numeric_limits<double>::quiet_NaN()};
    double orientation_x{std::numeric_limits<double>::quiet_NaN()};
    double orientation_y{std::numeric_limits<double>::quiet_NaN()};
    double orientation_z{std::numeric_limits<double>::quiet_NaN()};
    double angular_velocity_x{std::numeric_limits<double>::quiet_NaN()};
    double angular_velocity_y{std::numeric_limits<double>::quiet_NaN()};
    double angular_velocity_z{std::numeric_limits<double>::quiet_NaN()};
};

BOOST_DESCRIBE_STRUCT(TestRecord,
                      (cpp_sqlite::BaseTransferObject),
                      (simulation_time,
                       position_x, position_y, position_z,
                       velocity_x, velocity_y, velocity_z,
                       orientation_w, orientation_x, orientation_y, orientation_z,
                       angular_velocity_x, angular_velocity_y, angular_velocity_z));

#endif // TEST_RECORD_HPP
