#include <gtest/gtest.h>
#include "rotations.h"


TEST(rotations, rodrigues)
{
    cv::Vec3d r {0.7, -1.8, -0.9};
    cv::Matx33d R = rotmat(r);

    cv::Vec3d r2 = rodrigues(R);
    EXPECT_NEAR(norm(r, r2), 0, 1e-12);

    Quat q = quat(r);
    cv::Matx33d R2 = rotmat(q);
    EXPECT_NEAR(cv::norm(R - R2), 0, 1e-12);

    Quat q2 = quat(R);
    EXPECT_NEAR(norm((q.conj() * q2).vec()), 0, 1e-12);

    cv::Vec3d r3 = rodrigues(q2);
    EXPECT_NEAR(norm(r, r3), 0, 1e-12);

    AngleAxis aa = angleaxis(R);
    Quat q4 = quat(aa);

    cv::Vec3d r4 = rodrigues(q4);
    EXPECT_NEAR(norm(r, r4), 0, 1e-12);

    cv::Matx33d R3 = rotmat(aa);
    EXPECT_NEAR(cv::norm(R - R3), 0, 1e-12);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
