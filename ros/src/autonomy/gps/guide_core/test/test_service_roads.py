"""Tests for service roads (campus and access roads, driveways) in walk_costs."""

from guide_core.walk_costs import CostConfig, edge_cost, is_minor_service
import pytest


def test_service_road_factor_default():
    assert CostConfig().service_road_factor == 3.0


def test_service_road_costs_three_times():
    assert edge_cost(10.0, {'highway': 'service'}, {}, {}) == pytest.approx(30.0)


def test_service_road_with_sidewalk_costs_one_and_a_half():
    assert edge_cost(10.0, {'highway': 'service', 'sidewalk': 'both'}, {}, {}) \
        == pytest.approx(15.0)


def test_residential_road_unaffected():
    assert edge_cost(10.0, {'highway': 'residential'}, {}, {}) == pytest.approx(100.0)


def test_service_road_factor_below_one_rejected():
    with pytest.raises(ValueError):
        CostConfig(service_road_factor=0.5)


@pytest.mark.parametrize('tags, expected', [
    ({'highway': 'service', 'service': 'driveway'}, True),
    ({'highway': 'service', 'service': 'parking_aisle'}, True),
    ({'highway': 'service', 'service': 'drive-through'}, True),
    ({'highway': 'service', 'service': 'alley'}, False),
    ({'highway': 'service'}, False),
    ({'highway': 'residential', 'service': 'driveway'}, False),
])
def test_is_minor_service(tags, expected):
    assert is_minor_service(tags) is expected
