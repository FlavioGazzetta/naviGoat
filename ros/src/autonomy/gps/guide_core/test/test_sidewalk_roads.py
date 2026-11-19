"""Tests for roads whose pavement is tagged on the road (sidewalk=*)."""

from guide_core.walk_costs import CostConfig, edge_cost, has_sidewalk
import pytest


@pytest.mark.parametrize('value, expected', [
    ('both', True), ('left', True), ('right', True), ('yes', True),
    ('separate', False), ('no', False), ('none', False), (None, False),
])
def test_has_sidewalk(value, expected):
    tags = {'highway': 'residential'} if value is None else \
        {'highway': 'residential', 'sidewalk': value}
    assert has_sidewalk(tags) is expected


def test_sidewalk_road_factor_default():
    assert CostConfig().sidewalk_road_factor == 1.5


def test_minor_road_with_sidewalk_costs_one_and_a_half():
    assert edge_cost(10.0, {'highway': 'residential', 'sidewalk': 'both'}, {}, {}) \
        == pytest.approx(15.0)


def test_major_road_with_sidewalk_costs_one_and_a_half():
    assert edge_cost(10.0, {'highway': 'primary', 'sidewalk': 'left'}, {}, {}) \
        == pytest.approx(15.0)


def test_separately_mapped_sidewalk_keeps_road_factor():
    # The pavement is its own footway: use that, not the road.
    assert edge_cost(10.0, {'highway': 'residential', 'sidewalk': 'separate'}, {}, {}) \
        == pytest.approx(100.0)


def test_sidewalk_tag_ignored_on_footways():
    assert edge_cost(10.0, {'highway': 'footway', 'sidewalk': 'both'}, {}, {}) \
        == pytest.approx(10.0)


def test_sidewalk_road_in_tunnel():
    tags = {'highway': 'residential', 'sidewalk': 'both', 'tunnel': 'yes'}
    assert edge_cost(10.0, tags, {}, {}) == pytest.approx(10 * 1.5 * 3)


def test_sidewalk_road_factor_below_one_rejected():
    with pytest.raises(ValueError):
        CostConfig(sidewalk_road_factor=0.9)
