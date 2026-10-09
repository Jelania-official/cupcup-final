#include "sensor_time.hpp"

#include <cassert>
#include <cmath>
#include <cstdint>

int main()
{
    assert(cupcup::usableReceiptPose(0, .25));
    assert(!cupcup::usableReceiptPose(.250001, .1));
    assert(!cupcup::usableReceiptPose(.1, .250001));
    assert(!cupcup::usableReceiptPose(-.01, .1));
    assert(!cupcup::usableReceiptPose(.1, -.01));
    assert(!cupcup::usableReceiptPose(NAN, .1));
    assert(!cupcup::usableReceiptPose(.1, INFINITY));
    assert(cupcup::sameHeadTarget(0, 60, .5, 60.8));
    assert(!cupcup::sameHeadTarget(-4.36121, 29.9758, -1.48826, 50));
    assert(!cupcup::sameHeadTarget(0, 30, 0, 60));
    assert(!cupcup::sameHeadTarget(NAN, 60, 0, 60));
    cupcup::HeadTargetStability head;
    assert(head.age(10) == 0);
    head.observe(0, 20, 10);
    assert(!cupcup::usableBallReceiptGeometry(.1, .1, head.age(10.7)));
    assert(cupcup::usableBallReceiptGeometry(.1, .1, head.age(11)));
    head.observe(.6, 20, 11);  // sub-degree jitter retains the anchor
    assert(head.age(11) == 1);
    head.observe(1.2, 20, 11.1);  // accumulated small movement invalidates it
    assert(head.age(11.1) == 0);
    head.observe(1.2, 60, 12);
    assert(head.age(12.2) < .8);
    assert(!cupcup::usableBallReceiptGeometry(.3, .1, 2));
    assert(!cupcup::usableBallReceiptGeometry(.1, -.1, 2));
    assert(!cupcup::usableBallReceiptGeometry(.1, .1, NAN));
    head.observe(NAN, 60, 13);
    assert(head.age(14) == 0);
    head.observe(0, 20, 14);
    head.observe(0, 20, 1);  // receipt clock rewind starts a new episode
    assert(head.age(1) == 0);
    head.reset();
    assert(head.age(2) == 0);
    struct OfficialLocation { float x; float z; };
    struct ExtendedLocation { float x; float z; uint32_t stamp; };
    assert(cupcup::optionalStamp(OfficialLocation{0, 0}) == 0U);
    assert(cupcup::optionalStamp(ExtendedLocation{0, 0, 321U}) == 321U);
    uint32_t imageTime = 0;
    assert(cupcup::imageTimeMilliseconds(12, 345000000U, imageTime));
    assert(imageTime == 12345U);
    assert(!cupcup::imageTimeMilliseconds(0, 0U, imageTime));
    assert(!cupcup::imageTimeMilliseconds(1, 1000000000U, imageTime));

    assert(std::abs(cupcup::timestampAgeSeconds(1200U, 1000U) - 0.2) < 1e-9);
    assert(std::abs(cupcup::timestampAgeSeconds(1000U, 1200U) + 0.2) < 1e-9);
    assert(std::abs(cupcup::timestampAgeSeconds(50U, 0xfffffff0U) - 0.066) < 1e-9);
    cupcup::SensorHistory<int> history;
    history.push(1000U, 1);
    history.push(1020U, 2);
    int value = 0;
    double age = 99.0;
    assert(history.nearest(1020U, value, age) && value == 2 && age == 0.0);
    assert(history.nearest(1010U, value, age) && value == 1);
    assert(!history.nearest(1100U, value, age));
    assert(!history.nearest(0U, value, age));
    history.push(1010U, 99);
    assert(history.nearest(1020U, value, age) && value == 2);
    for (uint32_t stamp = 1040U; stamp <= 1800U; stamp += 20U) history.push(stamp, 3);
    assert(!history.nearest(1020U, value, age));
    history.push(100U, 4);
    assert(history.nearest(100U, value, age) && value == 4);
    assert(!history.nearest(1800U, value, age));
    cupcup::SensorHistory<int> wrapping;
    wrapping.push(0xfffffff0U, 5);
    wrapping.push(20U, 6);
    assert(wrapping.nearest(20U, value, age) && value == 6);
    assert(wrapping.nearest(0xfffffff0U, value, age) && value == 5);
    return 0;
}
