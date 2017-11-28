# pylint: disable=missing-docstring

import math
import plan_route


def test_main_Frogmore():
    a = plan_route.MapPosition(latitude=50.280278, longitude=-3.781039)
    b = plan_route.MapPosition(latitude=50.269988, longitude=-3.743058)
    direction = plan_route.main(a, b)
    assert math.isclose(direction[1], 1.6195231581279383)


def test_main_Salcombe2():
    a = plan_route.MapPosition(latitude=50.280278, longitude=-3.781039)
    b = plan_route.MapPosition(latitude=50.262894, longitude=-3.812115)
    direction = plan_route.main(a, b)
    assert math.isclose(direction[1], 1.6195231581279383)


def test_main_Salcombe3():
    a = plan_route.MapPosition(latitude=50.280278, longitude=-3.781039)
    b = plan_route.MapPosition(latitude=50.237485, longitude=-3.778059)
    direction = plan_route.main(a, b)
    assert math.isclose(direction[1], 1.6195231581279383)


def test_main_Salcombe4():
    a = plan_route.MapPosition(latitude=50.280278, longitude=-3.781039)
    b = plan_route.MapPosition(latitude=50.244539, longitude=-3.802108)
    direction = plan_route.main(a, b)
    assert math.isclose(direction[1], 1.6195231581279383)


def test_main_Salcombe5():
    a = plan_route.MapPosition(latitude=50.280278, longitude=-3.781039)
    b = plan_route.MapPosition(latitude=50.251130, longitude=-3.809463)
    direction = plan_route.main(a, b)
    assert math.isclose(direction[1], 1.6195231581279383)


def test_main_Salcombe6():
    a = plan_route.MapPosition(latitude=50.280278, longitude=-3.781039)
    b = plan_route.MapPosition(latitude=50.257111, longitude=-3.812122)
    direction = plan_route.main(a, b)
    assert math.isclose(direction[1], 1.6195231581279383)


def test_main_Salcombe12():
    a = plan_route.MapPosition(latitude=50.280278, longitude=-3.781039)
    b = plan_route.MapPosition(latitude=50.256824, longitude=-3.812059)
    direction = plan_route.main(a, b)
    assert math.isclose(direction[1], 1.6195231581279383)


def test_main_Salcombe14():
    a = plan_route.MapPosition(latitude=50.280278, longitude=-3.781039)
    b = plan_route.MapPosition(latitude=50.256744, longitude=-3.812049)
    direction = plan_route.main(a, b)
    print(direction)
    assert math.isclose(direction[1], 1.6195231581279383)


def test_main_Salcombe13():
    a = plan_route.MapPosition(latitude=50.256688, longitude=-3.812043)
    b = plan_route.MapPosition(latitude=50.235181, longitude=-3.775986)
    direction = plan_route.main(a, b)
    assert math.isclose(direction[1], 2.8916234585323193)
