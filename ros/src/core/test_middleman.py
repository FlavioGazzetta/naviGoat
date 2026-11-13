import unittest
from unittest import mock

import middleman
from middleman import Middleman, Mode, classify, qwen_interpreter

# (said, mode, target); mode None means no mode change
CASES = [
    ('stop', Mode.IDLE, ''),
    ('pete stop following me', Mode.IDLE, ''),
    ("don't follow me", Mode.IDLE, ''),
    ('follow this person', Mode.FOLLOW, ''),
    ('keep following him', Mode.FOLLOW, ''),
    ('pete follow Alice', Mode.FOLLOW, 'alice'),
    ('follow me to the kitchen', Mode.LOCATION, 'kitchen'),
    ('take me to the bathroom', Mode.LOCATION, 'bathroom'),
    ('take me to the toilet', Mode.LOCATION, 'bathroom'),
    ('I need the bathroom', Mode.LOCATION, 'bathroom'),
    ('take me outside', Mode.LOCATION, 'exit'),
    ('I have to pee', Mode.LOCATION, 'bathroom'),
    ('take me to the chair', Mode.OBJECT, 'chair'),
    ('bring me to the sofa', Mode.OBJECT, 'couch'),
    ('gps take me to 12 Main Street', Mode.GPS, '12 main street'),
    ('Hey Pete, GPS take me to South Kensington tube station', Mode.GPS, 'south kensington tube station'),
    ('manual mode', Mode.MANUAL, ''),
    ('pete enter manual mode', Mode.MANUAL, ''),
    ('switch to manual control', Mode.MANUAL, ''),
    ('let me drive', Mode.MANUAL, ''),
    ('help', Mode.HELP, ''),
    ("I'm lost", Mode.HELP, ''),
    ("I'm not lost", None, ''),
    ('hmm the thing', None, ''),
]

# said to the robot but about what the camera sees: sent as it was said to perception, no mode
PERCEPTION = ["what's in front of me", 'what is around me', 'is the light on?', "I'd like a coffee"]


class ClassifyTest(unittest.TestCase):
    def test_cases(self):
        for text, mode, target in CASES:
            with self.subTest(text):
                decision = classify(text)
                self.assertEqual((decision.mode, decision.target), (mode, target))

    def test_every_decision_has_something_to_say_or_forward(self):
        for text, mode, _ in CASES:
            with self.subTest(text):
                decision = classify(text)
                self.assertTrue(decision.reply or decision.forward)

    def test_scene_requests_go_to_perception(self):
        for text in PERCEPTION:
            with self.subTest(text):
                decision = classify(text)
                self.assertIsNone(decision.mode)
                self.assertEqual(decision.forward, text.lower().replace('?', ''))

    def test_where_is_x_takes_you_to_x(self):
        for text, mode, target in (('where is the cup', Mode.OBJECT, 'cup'), ('where are the glasses', Mode.OBJECT, 'glasses'),
                                   ('where is the bathroom', Mode.LOCATION, 'bathroom'), ('I lost my phone', Mode.OBJECT, 'cell phone'),
                                   ('find my keys', Mode.OBJECT, 'keys'), ('help me find my keys', Mode.OBJECT, 'keys'),
                                   ('where is the bottle', Mode.OBJECT, 'bottle')):
            with self.subTest(text):
                decision = classify(text)
                self.assertEqual((decision.mode, decision.target), (mode, target))
                self.assertEqual(decision.reply, f'Going to the {target}.')
                self.assertFalse(decision.forward)

    def test_where_is_never_says_it_cannot_look_for_things(self):
        for text in ('where is the cup', 'where is the piano', 'is there a chair nearby', 'do you see my keys', 'where'):
            with self.subTest(text):
                self.assertNotIn("Can't look for", classify(text).reply)

    def test_where_is_an_unknown_thing_is_not_guessed(self):
        self.assertEqual(classify('where is the piano').reply, "I don't know where the piano is.")

    def test_where_is_never_reaches_qwen(self):
        with mock.patch.object(middleman.urllib.request, 'urlopen', side_effect=AssertionError('Qwen was asked')):
            self.assertEqual(classify('pete where is the cup', interpreter=qwen_interpreter()).target, 'cup')

    def test_i_need_to_find_a_place_still_takes_you_there(self):
        self.assertEqual(classify('I need to find the bathroom').mode, Mode.LOCATION)

    def test_unknown_location_and_object_get_a_reply(self):
        for text, reply in (('take me to the cafeteria', "I don't know where the cafeteria is."),
                            ('take me to the piano', "I don't know where the piano is."),
                            ('take me to 12 Main Street', "I don't know where 12 main street is."),
                            ('take me to the train station', "I don't know where the train station is."),
                            ('take me home', "I don't know where home is.")):
            with self.subTest(text):
                decision = classify(text)
                self.assertIsNone(decision.mode)
                self.assertFalse(decision.forward)
                self.assertEqual(decision.reply, reply)

    def test_a_misheard_word_still_matches_but_a_different_word_does_not(self):
        self.assertEqual((classify('take me to the bathrom').mode, classify('take me to the bathrom').target), (Mode.LOCATION, 'bathroom'))
        self.assertEqual(classify('take me to the chairs').target, 'chair')
        self.assertIsNone(classify('take me to the piano').mode)
        for plural, singular in (('boxes', 'box'), ('buses', 'bus'), ('sofas', 'couch')):
            self.assertEqual(classify(f'take me to the {plural}').target, singular)

    def test_gps_needs_the_word_gps(self):
        self.assertIsNone(classify('take me to 12 Main Street').mode)
        self.assertEqual(classify('gps take me to 12 Main Street').mode, Mode.GPS)
        self.assertIn('GPS to where', classify('gps').reply)

    def test_two_requests_get_a_reply(self):
        decision = classify('take me to the chair and then the table')
        self.assertIsNone(decision.mode)
        self.assertIn('One at a time', decision.reply)

    def test_beep_only_when_there_is_no_spoken_reply(self):
        beeps = []
        bot = Middleman(on_ack=lambda: beeps.append(1))
        for text in ('take me to the bathroom', 'follow alice', 'manual mode', 'stop', 'take me to the unicorn', 'where is the cup'):
            bot.handle(text)
        self.assertEqual(beeps, [])
        bot.handle("what's in front of me")
        self.assertEqual(beeps, [1])

    def test_wake_word_required(self):
        self.assertIsNone(classify('follow alice', require_wake_word=True))
        self.assertEqual(classify('hey pete follow alice', require_wake_word=True).mode, Mode.FOLLOW)
    def test_saved_home(self):
        saved = {'home': (Mode.GPS, '12 main street')}
        for text in ('take me home', 'take me to home', 'go home'):
            with self.subTest(text):
                decision = classify(text, saved=saved)
                self.assertEqual((decision.mode, decision.target), (Mode.GPS, '12 main street'))

    def test_empty(self):
        self.assertIsNone(classify('  '))
        self.assertIsNone(classify('pete'))


class MiddlemanTest(unittest.TestCase):
    def test_modes_change_and_stop_returns_to_idle(self):
        bot = Middleman()
        bot.handle('follow alice')
        self.assertEqual(bot.mode, Mode.FOLLOW)
        bot.handle('stop')
        self.assertEqual(bot.mode, Mode.IDLE)

    def test_manual_mode_until_stop(self):
        bot = Middleman()
        bot.handle('pete enter manual mode')
        self.assertEqual(bot.mode, Mode.MANUAL)
        bot.handle("what's in front of me")
        self.assertEqual(bot.mode, Mode.MANUAL)
        bot.handle('stop')
        self.assertEqual(bot.mode, Mode.IDLE)

    def test_exit_phrases_leave_manual_and_follow(self):
        for enter, leave, reply in (('manual mode', 'exit manual mode', 'Manual mode off'), ('follow alice', 'stop following', 'Stopped following'), ('follow alice', 'leave follow mode', 'Stopped following'), ('manual mode', 'stop', 'Manual mode off')):
            with self.subTest(enter + ' / ' + leave):
                bot = Middleman()
                bot.handle(enter)
                decision = bot.handle(leave)
                self.assertTrue(decision.stop)
                self.assertEqual(bot.mode, Mode.IDLE)
                self.assertIn(reply, decision.reply)

    def test_exit_manual_mode_does_not_start_manual_mode(self):
        self.assertTrue(classify('exit manual mode').stop)
        self.assertEqual(classify('manual mode').mode, Mode.MANUAL)

    def test_a_goal_mode_ends_when_the_robot_arrives(self):
        bot = Middleman()
        bot.handle('take me to the bathroom')
        self.assertEqual(bot.mode, Mode.LOCATION)
        bot.finish()
        self.assertEqual(bot.mode, Mode.IDLE)

    def test_unknown_and_perception_keep_current_mode(self):
        bot = Middleman()
        bot.handle('follow alice')
        bot.handle('take me to the cafeteria')
        bot.handle("what's in front of me")
        self.assertEqual(bot.mode, Mode.FOLLOW)

    def test_failed_guide_falls_back_to_a_stored_location(self):
        bot = Middleman()
        bot.handle('take me to the sink')
        self.assertEqual(bot.mode, Mode.OBJECT)
        decision = bot.fail()
        self.assertEqual((decision.mode, decision.target), (Mode.LOCATION, 'kitchen'))

    def test_object_not_found_is_spoken_and_goes_idle(self):
        bot = Middleman()
        bot.handle('take me to the chair')
        decision = bot.not_found('chair')
        self.assertIsNone(decision.mode)
        self.assertEqual(decision.reply, "I couldn't find the chair.")
        self.assertEqual(bot.mode, Mode.IDLE)

    def test_object_not_found_uses_a_stored_location_when_there_is_one(self):
        bot = Middleman()
        bot.handle('take me to the sink')
        decision = bot.not_found('sink')
        self.assertEqual((decision.mode, decision.target), (Mode.LOCATION, 'kitchen'))
        self.assertIn("No sink", decision.reply)

    def test_object_not_found_about_something_else_is_ignored(self):
        bot = Middleman()
        bot.handle('take me to the chair')
        self.assertIsNone(bot.not_found('cup'))  # a late report about an earlier search
        self.assertEqual(bot.mode, Mode.OBJECT)
        bot.handle('stop')
        self.assertIsNone(bot.not_found('chair'))  # nothing is being searched for any more

    def test_the_wake_word_can_come_after_the_request(self):
        for text, mode, target in (('Take me to the bathroom. Pete.', Mode.LOCATION, 'bathroom'),
                                   ('follow me, Pete', Mode.FOLLOW, ''), ('stop, Pete.', Mode.IDLE, '')):
            with self.subTest(text):
                decision = classify(text)
                self.assertEqual((decision.mode, decision.target), (mode, target))

    def test_a_greeting_and_the_name_alone_is_nothing_to_act_on(self):
        for text in ('Hello, Pete.', 'Hey Pete.', 'Pete.'):
            with self.subTest(text):
                self.assertIsNone(classify(text))

    def test_follow_follow_is_not_a_name(self):
        decision = classify('Pete, follow, follow.')
        self.assertEqual((decision.mode, decision.target, decision.reply), (Mode.FOLLOW, '', 'Following.'))

    def test_names_whisper_writes_for_pret_and_the_library(self):
        for text, place in (('go to press', 'pret'), ('go to Abdu-Salam Library', 'abdus salam library'),
                            ('take me to the library', 'abdus salam library'), ('take me to the scampton building', 'skempton building')):
            with self.subTest(text):
                self.assertEqual(classify(text).target, place)

    def test_the_language_model_is_only_asked_when_the_rules_do_not_understand(self):
        asked = []

        def fake(text, minimum=0.7):
            asked.append(text)
            return middleman.Decision(Mode.LOCATION, 'goto', 'pret', text, reply='Going to the pret.')
        self.assertEqual(classify('pete I am hungry', interpreter=fake).target, 'pret')
        self.assertEqual(asked, ['i am hungry'])
        for text in ('pete follow me', 'pete stop', 'pete help', 'pete take me to the bathroom', 'pete where is the cup', 'pete what is in front of me'):
            classify(text, interpreter=fake)
        self.assertEqual(asked, ['i am hungry'])  # none of those reached it

    def test_one_unfamiliar_word_is_refused_not_guessed_by_meaning(self):
        asked = []

        def fake(text, minimum=0.7):
            asked.append(text)
            return middleman.Decision(Mode.LOCATION, 'goto', 'prayer room', text, reply='Going to the prayer room.')
        self.assertEqual(classify('pete go to prud', interpreter=fake).reply, "I don't know where the prud is.")
        self.assertEqual(asked, [])  # a mis-heard name is not sent to the language model
        self.assertEqual(classify('pete bring me to the place with the books', interpreter=fake).target, 'prayer room')
        self.assertEqual(len(asked), 1)  # a description is

    def test_an_unknown_place_stays_refused_when_the_language_model_is_not_sure(self):
        self.assertIn("I don't know where", classify("pete take me to the dean's office", interpreter=lambda text, minimum=0.7: None).reply)

    def test_asking_what_is_around_is_sent_to_perception_as_a_description(self):
        for text in ('what is the scene in front of me', 'describe the scene around me', 'what is around me', 'what is the scene near me',
                     'describe the scene', 'what do you see', 'describe my surroundings', 'what is nearby', "what's the scenery", 'look around'):
            with self.subTest(text):
                decision = classify('pete ' + text)
                self.assertTrue(decision.forward)
                self.assertTrue(middleman.DESCRIBE.search(decision.forward))

    def test_a_search_is_not_taken_for_a_description(self):
        for text in ('where is the chair', 'is there a chair in front of me', 'do you see my keys', 'find my keys'):
            with self.subTest(text):
                decision = classify('pete ' + text)
                self.assertFalse(decision.forward)
                self.assertTrue(decision.reply)

    def test_vague_requests_reach_the_language_model_but_commands_and_descriptions_do_not(self):
        asked = []

        def fake(text, minimum=0.7):
            asked.append(text)
            return middleman.Decision(Mode.LOCATION, 'goto', 'pret', text, reply='Going to the pret.')
        for text in ("I'm hungry", 'get me a coffee', 'can you get me something to eat', "I'd like something to drink", 'I want a coffee'):
            with self.subTest(text):
                self.assertEqual(classify('pete ' + text, interpreter=fake).target, 'pret')
        self.assertEqual(len(asked), 5)
        for text in ("what's around me", 'what do you see', 'follow me', 'stop', 'help', 'take me to the bathroom', 'where is the cup'):
            classify('pete ' + text, interpreter=fake)
        self.assertEqual(len(asked), 5)  # none of those was sent to it

    def test_failed_guide_without_fallback_goes_idle(self):
        bot = Middleman()
        bot.handle('take me to the chair')
        self.assertIsNone(bot.fail())
        self.assertEqual(bot.mode, Mode.IDLE)


if __name__ == '__main__':
    unittest.main()
