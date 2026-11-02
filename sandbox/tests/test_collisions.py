"""Tests for the contact-message logic behind the collision metric (the ROS node only feeds it)."""
import pytest

from sewer_eval.collisions import CollisionCounter, category, partner_model

R = 'sewer_robot::base_link::base_link_collision'


def test_partner_is_the_obstacle_model_in_either_order():
    assert partner_model(R, 'walls::link::c3') == 'walls'
    assert partner_model('debris_B2::link::c0', R) == 'debris_B2'


def test_floor_ground_and_robot_only_pairs_are_ignored():
    assert partner_model('sewer_robot::left_wheel_link::left_wheel_link_collision', 'floor::link::c') is None
    assert partner_model(R, 'ground::link::c') is None
    assert partner_model(R, 'sewer_robot::left_wheel_link::left_wheel_link_collision') is None
    assert partner_model('walls::link::c1', 'debris_B1::link::c0') is None


def test_categories():
    assert category('walls') == 'wall' and category('debris_B1') == 'debris' and category('obstacle_box') == 'other'


def test_a_long_scrape_is_one_collision_and_a_new_touch_after_the_gap_is_another():
    c = CollisionCounter(gap=1.0)
    for k in range(40):                                   # 2 s of scraping the walls at 20 Hz
        c.update(10.0 + 0.05 * k, {'walls'})
    assert len(c.events) == 1
    assert c.contact_time == pytest.approx(1.95, abs=0.01)
    c.update(14.0, {'walls'})                             # touches again 2 s later
    assert len(c.events) == 2
    c.update(14.5, {'debris_B1'})                         # a different obstacle at once counts separately
    assert [m for _, m in c.events] == ['walls', 'walls', 'debris_B1']


def test_no_contact_means_no_events_and_summary_counts_by_category():
    c = CollisionCounter()
    assert c.update(1.0, set()) == [] and c.summary()['total'] == 0
    c.update(2.0, {'walls', 'debris_B2'})
    s = c.summary()
    assert (s['total'], s['wall'], s['debris'], s['other']) == (2, 1, 1, 0)
