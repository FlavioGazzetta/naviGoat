import io
import json
import threading
import unittest
import urllib.error
from unittest import mock

import middleman
from middleman import Middleman, Mode, classify, qwen_interpreter

PLACES = ['bathroom', 'kitchen', 'exit']


def fake_qwen(mode, target='', match='', confidence=0.0):
    """What urlopen returns when Qwen answers with this mode, target, closest stored match and confidence."""
    answer = {'mode': mode, 'target': target, 'match': match, 'confidence': confidence}
    body = json.dumps({'message': {'content': json.dumps(answer)}}).encode()
    return mock.patch.object(middleman.urllib.request, 'urlopen', return_value=io.BytesIO(body))


def understand(mode, target='', text='pete anything', match='', confidence=0.0):
    with fake_qwen(mode, target, match, confidence):
        return classify(text, PLACES, interpreter=qwen_interpreter(PLACES))


class QwenTest(unittest.TestCase):
    def check(self, decision, mode, target=''):
        self.assertEqual((decision.mode, decision.target), (mode, target))

    def test_locations_and_objects(self):
        self.check(understand('location', 'toilet'), Mode.LOCATION, 'bathroom')
        self.check(understand('location', 'the kitchen'), Mode.LOCATION, 'kitchen')
        self.check(understand('object', 'the sofa'), Mode.OBJECT, 'couch')

    def test_something_similar_links_to_the_closest_stored_name(self):
        self.check(understand('object', 'mug', match='cup', confidence=0.9), Mode.OBJECT, 'cup')
        self.check(understand('location', 'canteen', match='kitchen', confidence=0.8), Mode.LOCATION, 'kitchen')
        self.check(understand('location', 'the loo', match='bathroom', confidence=0.95), Mode.LOCATION, 'bathroom')

    def test_low_confidence_or_no_match_is_not_found(self):
        decision = understand('object', 'unicorn', match='cat', confidence=0.3)
        self.assertIsNone(decision.mode)
        self.assertIn('unicorn', decision.reply)
        self.assertIsNone(understand('location', 'cafeteria', match='kitchen', confidence=0.69).mode)
        self.assertIsNone(understand('location', 'cafeteria', match='', confidence=0.9).mode)

    def test_a_match_that_is_not_stored_or_a_bad_confidence_is_ignored(self):
        self.assertIsNone(understand('object', 'unicorn', match='horse', confidence=0.99).mode)
        self.assertIsNone(understand('object', 'mug', match='cup', confidence='very').mode)

    def test_unknown_location_and_object_get_a_reply(self):
        decision = understand('location', 'cafeteria')
        self.assertIsNone(decision.mode)
        self.assertEqual(decision.reply, 'Unknown location: cafeteria.')
        decision = understand('object', 'unicorn')
        self.assertIsNone(decision.mode)
        self.assertIn('unicorn', decision.reply)

    def test_gps_only_when_the_user_says_gps(self):
        self.check(understand('gps', '12 Main Street', 'pete gps take me to 12 main street'), Mode.GPS, '12 main street')
        decision = understand('gps', '12 Main Street', 'pete take me to 12 main street')
        self.assertIsNone(decision.mode)
        self.assertIn('GPS', decision.reply)

    def test_follow_another_person_with_or_without_a_name(self):
        self.check(understand('follow', text='pete follow that person'), Mode.FOLLOW)
        self.check(understand('follow', 'Alice', 'pete follow alice'), Mode.FOLLOW, 'alice')
    def test_perception_is_forwarded_as_said(self):
        decision = understand('perception', text="what's ahead")
        self.assertIsNone(decision.mode)
        self.assertEqual(decision.forward, "what's ahead")

    def test_manual_mode(self):
        self.check(understand('manual'), Mode.MANUAL)

    def test_stop_and_help(self):
        self.assertTrue(understand('stop').stop)
        self.check(understand('help'), Mode.HELP)

    def test_stop_and_help_never_wait_for_qwen(self):
        with mock.patch.object(middleman.urllib.request, 'urlopen', side_effect=AssertionError('Qwen was asked')):
            self.assertTrue(classify('pete stop', PLACES, interpreter=qwen_interpreter(PLACES)).stop)
            self.check(classify("pete i'm lost", PLACES, interpreter=qwen_interpreter(PLACES)), Mode.HELP)

    def test_beep_for_a_perception_request_from_qwen(self):
        beeps = []
        with fake_qwen('perception'):
            Middleman(PLACES, interpreter=qwen_interpreter(PLACES), on_ack=lambda: beeps.append(1)).handle("pete what's ahead")
        self.assertEqual(beeps, [1])

    def test_rules_decide_when_qwen_is_unreachable_or_unsure(self):
        with mock.patch.object(middleman.urllib.request, 'urlopen', side_effect=urllib.error.URLError('no ollama')):
            self.check(classify('pete take me to the bathroom', PLACES, interpreter=qwen_interpreter(PLACES)), Mode.LOCATION, 'bathroom')
        with fake_qwen('unknown'):
            self.check(classify('pete follow alice', PLACES, interpreter=qwen_interpreter(PLACES)), Mode.FOLLOW, 'alice')
        with mock.patch.object(middleman.urllib.request, 'urlopen', return_value=io.BytesIO(b'{"message": {"content": "not json"}}')):
            self.check(classify('pete follow alice', PLACES, interpreter=qwen_interpreter(PLACES)), Mode.FOLLOW, 'alice')

    def test_stop_is_handled_while_qwen_is_slow_and_cancels_the_slow_request(self):
        release = threading.Event()
        asked = threading.Event()

        def slow(text):
            asked.set()
            release.wait(5)
            return classify_with_mode('location', 'kitchen', text)

        def classify_with_mode(mode, target, text):
            return middleman.decision_from_mode(mode, target, text, PLACES, {})

        bot = Middleman(PLACES, interpreter=slow)
        results = []
        thread = threading.Thread(target=lambda: results.append(bot.handle('pete take me to the kitchen')))
        thread.start()
        self.assertTrue(asked.wait(2))
        stop = bot.handle('pete stop')  # while the first request is still with the interpreter
        self.assertTrue(stop.stop)
        self.assertEqual(bot.mode, Mode.IDLE)
        release.set()
        thread.join(5)
        self.assertEqual(results, [None])  # dropped: it cannot start a mode after the stop
        self.assertEqual(bot.mode, Mode.IDLE)

    def test_a_slow_request_that_finishes_without_a_stop_still_applies(self):
        bot = Middleman(PLACES, interpreter=lambda text: middleman.decision_from_mode('location', 'kitchen', text, PLACES, {}))
        self.assertEqual(bot.handle('pete take me to the kitchen').mode, Mode.LOCATION)

    def test_the_middleman_keeps_its_mode_with_an_interpreter(self):
        with fake_qwen('location', 'toilet'):
            bot = Middleman(PLACES, interpreter=qwen_interpreter(PLACES))
            bot.handle("pete i'd really like to wash my hands")
        self.assertEqual(bot.mode, Mode.LOCATION)


if __name__ == '__main__':
    unittest.main()
