from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.messages.flat.QuickChatSelection import QuickChatSelection
from rlbot.utils.structures.game_data_struct import GameTickPacket

from util.ball_prediction_analysis import find_slice_at_time
from util.boost_pad_tracker import BoostPadTracker
from util.drive import steer_toward_target
from util.sequence import Sequence, ControlStep
from util.vec import Vec3

from math import pi


class MyBot(BaseAgent):

    '''Mode Enums'''
    MODE_BALLCHASE = 0
    MODE_RETREAT = 1
    MODE_AERIAL_RECOVER = 2

    def __init__(self, name, team, index):
        super().__init__(name, team, index)
        self.active_sequence: Sequence = None
        self.boost_pad_tracker = BoostPadTracker()
        self.mode = self.MODE_BALLCHASE
        self.last_flip = -2

    def initialize_agent(self):
        # Set up information about the boost pads now that the game is active and the info is available
        self.boost_pad_tracker.initialize_boosts(self.get_field_info())

    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
        """
        This function will be called by the framework many times per second. This is where you can
        see the motion of the ball, etc. and return controls to drive your car.
        """

        # Keep our boost pad info updated with which pads are currently active
        self.boost_pad_tracker.update_boost_status(packet)

        # This is good to keep at the beginning of get_output. It will allow you to continue
        # any sequences that you may have started during a previous call to get_output.
        if self.active_sequence is not None and not self.active_sequence.done:
            controls = self.active_sequence.tick(packet)
            if controls is not None:
                return controls

        # Gather some information about our car, the ball, and the field
        my_car = packet.game_cars[self.index]
        car_location = Vec3(my_car.physics.location)
        car_velocity = Vec3(my_car.physics.velocity)
        ball_location = Vec3(packet.game_ball.physics.location)
        ball_velocity = Vec3(packet.game_ball.physics.location)
        field_info = self.get_field_info()
        our_goal_location = self.get_team_goal_pos(field_info)
        ang_to_goal = car_velocity.ang_to(our_goal_location.flat() - car_location)
        ang_to_ball = car_velocity.ang_to(ball_location.flat() - car_location)

        ball_goalside_location = self.get_future_ball_location(packet, 1.5)

        # Setting mode
        # If we're ballchasing, we want to continue ballchasing unless we are no longer goalside
        #   in which case we begin retreating
        # If we're retreating, we want to continue retreating until we are goalside
        #   in which case we begin ballchasing
        if self.mode == self.MODE_BALLCHASE:
            if not self.between_ball_and_goal(ball_location, car_location, field_info):
                self.mode = self.MODE_RETREAT
        elif self.mode == self.MODE_RETREAT:
            if self.between_ball_and_goal(ball_goalside_location, car_location, field_info):    
                self.mode = self.MODE_BALLCHASE
        # force into ballchase mode if in defensive position
        if (our_goal_location - car_location).length() < 700:
            self.mode = self.MODE_BALLCHASE

        if self.mode == self.MODE_BALLCHASE:
            self.renderer.draw_string_2d(0, 0, 2, 2, 'Target: ball', self.renderer.white())
            target_location = ball_location
            dist_to_target = (target_location - car_location).length()
            est_time_to_target = dist_to_target / car_velocity.length()
            target_location = self.get_future_ball_location(packet, est_time_to_target)
            self.renderer.draw_line_3d(ball_location, target_location, self.renderer.cyan())
        else: 
            self.renderer.draw_string_2d(0, 0, 2, 2, 'Target: goal', self.renderer.white())
            target_location = our_goal_location.flat()
            dist_to_target = (target_location - car_location).length()
            est_time_to_target = dist_to_target / car_velocity.length()
            self.renderer.draw_line_3d(ball_location, ball_goalside_location, self.renderer.cyan())

        ang_to_target = car_velocity.ang_to(target_location.flat() - car_location)

        # Draw some things to help understand what the bot is thinking
        self.renderer.draw_line_3d(car_location, target_location, self.renderer.white())
        self.renderer.draw_string_3d(target_location, 1, 1, f'Estimated time to target: {est_time_to_target:.1f}', self.renderer.white())
        self.renderer.draw_rect_3d(target_location, 8, 8, True, self.renderer.cyan(), centered=True)
        

        controls = SimpleControllerState()
        controls.steer = steer_toward_target(my_car, target_location)

        brake = False

        controls.throttle = 1.0
        if self.mode == self.MODE_RETREAT:
            if (target_location - Vec3(car_location)).length() < 800 and car_velocity.length() > 700:
                self.renderer.draw_string_2d(0, 60, 2, 2, f'Braking!', self.renderer.white())
                brake = True
                controls.throttle = -1.0

        if self.mode == self.MODE_BALLCHASE and est_time_to_target < 0.3:
            if ball_location[2] > 200: 
                return self.begin_jump(packet)
            else:
                return self.begin_front_flip(packet)
        
        # We only want to boost if we're going in the direction of the target
        self.renderer.draw_string_2d(0, 30, 2, 2, f'Angle to target: {round(ang_to_target, 1)}', self.renderer.white())

        if ang_to_target < pi / 5 and car_velocity.length() < 2200 and not brake: # We don't want to boost if the car is max speed
            # flip if no boost
            if my_car.boost == 0 and est_time_to_target > 2 and car_velocity.length() > 800:
                if packet.game_info.seconds_elapsed - self.last_flip > 1.5: # ratelimit flips
                    return self.begin_front_flip(packet)
            controls.boost = 1

        # We drift if we're in the wrong direction
        if ang_to_target > 4 / 5 * pi:
            controls.handbrake = 1

        return controls

    def begin_front_flip(self, packet):
        # Do a front flip. We will be committed to this for a few seconds and the bot will ignore other
        # logic during that time because we are setting the active_sequence.
        self.active_sequence = Sequence([
            ControlStep(duration=0.05, controls=SimpleControllerState(jump=True)),
            ControlStep(duration=0.05, controls=SimpleControllerState(jump=False)),
            ControlStep(duration=0.2, controls=SimpleControllerState(jump=True, pitch=-1)),
            ControlStep(duration=0.8, controls=SimpleControllerState()),
        ])

        self.last_flip = packet.game_info.seconds_elapsed

        # Return the controls associated with the beginning of the sequence so we can start right away.
        return self.active_sequence.tick(packet)

    def begin_jump(self, packet):
        self.active_sequence = Sequence([
            ControlStep(duration=0.1, controls=SimpleControllerState(jump=True)),
            ControlStep(duration=0.05, controls=SimpleControllerState()),
        ])

        # Return the controls associated with the beginning of the sequence so we can start right away.
        return self.active_sequence.tick(packet)

    # Bot should only go towards the ball if the bot is between the ball and our goal. 
    def between_ball_and_goal(self, ball_location, car_location, field_info):
        our_goal = self.get_team_goal_pos(field_info)
        return ball_location.y < car_location.y < our_goal.y

    # Get the location of our team's goal 
    def get_team_goal_pos(self, field_info):
        goals = field_info.goals
        for goal in goals:
            if goal.team_num == self.team:
                return Vec3(goal.location)

    def get_future_ball_location(self, packet, seconds):
        # Ball location prediction
        ball_prediction = self.get_ball_prediction_struct()  # This can predict bounces, etc
        ball_in_future = find_slice_at_time(ball_prediction, packet.game_info.seconds_elapsed + seconds)

        # ball_in_future might be None if we don't have an adequate ball prediction right now, like during
        # replays, so check it to avoid errors.
        ball_future_location = Vec3(packet.game_ball.physics.location)
        if ball_in_future is not None:
            ball_future_location = Vec3(ball_in_future.physics.location)        
        return ball_future_location
                



        
