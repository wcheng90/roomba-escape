# activate project environment
# include these lines of code in any future scripts/notebooks
#---
import Pkg
if !haskey(Pkg.installed(), "AA228FinalProject")
    jenv = joinpath(dirname(@__FILE__()), ".") # this assumes the notebook is in the same dir
    # as the Project.toml file, which should be in top level dir of the project. 
    # Change accordingly if this is not the case.
    Pkg.activate(jenv)
end
#---

# import necessary packages
using AA228FinalProject
using POMDPs
using POMDPPolicies
using BeliefUpdaters
using ParticleFilters
using POMDPSimulators
using Cairo
using Gtk
using Random
using Printf

sensor = Bumper() #Lidar() # or Bumper() for the bumper version of the environment
config = 1 # 1,2, or 3
m = RoombaPOMDP(sensor=sensor, mdp=RoombaMDP(config=config));

num_particles = 2000
resampler = LidarResampler(num_particles, LowVarianceResampler(num_particles))
# for the bumper environment
# resampler = BumperResampler(num_particles)

spf = SimpleParticleFilter(m, resampler)

v_noise_coefficient = 2.0
om_noise_coefficient = 0.5

belief_updater = RoombaParticleFilter(spf, v_noise_coefficient, om_noise_coefficient);

# Define the policy to test
mutable struct ToEnd <: Policy
    ts::Int64 # to track the current time-step.
end

# extract goal for heuristic controller
goal_xy = get_goal_xy(m)

# define a new function that takes in the policy struct and current belief,
# and returns the desired action
function POMDPs.action(p::ToEnd, b::ParticleCollection{RoombaState})
    
    # spin around to localize for the first 25 time-steps
#   if p.ts < 25
#        p.ts += 1
#        return RoombaAct(0.,1.0) # all actions are of type RoombaAct(v,om)
#    end
    
    if p.ts < 10
	    p.ts += 1
        if AA228FinalProject.wall_contact(m, particles(b)[1])
#            println("True")
            return RoombaAct(1.0, pi) 
        else
#            println("False")
    	    return RoombaAct(10., 0.0)
        end
    end
    p.ts += 1    
    # compute mean belief of a subset of particles
    s = mean(b)

    # Need to figure out how to grab a single particle
#    @printf "%s \n" typeof(b)    
    
    # compute the difference between our current heading and one that would
    # point to the goal
    goal_x, goal_y = goal_xy
#    @printf "Goal X: %d; Goal Y: %d \n" goal_x goal_y
    x,y,th = s[1:3]
#    @printf "Current X: %d; Current Y: %d; Current Theta: %d \n" x y th
    del_x = goal_x - x
    del_y = goal_y - y
#    if abs(del_x) < 1 
#        ang_to_goal = atan(del_y, 0)
#    elseif abs(del_y) < 1
#        ang_to_goal = atan(0, del_x)
#    else
#        ang_to_goal = atan(del_y, del_x)
#    end
    ang_to_goal = atan(goal_y - y, goal_x - x)

#    @printf "Delta X: %d; Delta Y: %d \n" del_x del_y

    if AA228FinalProject.wall_contact(m, particles(b)[1])
        ang_to_goal = ang_to_goal + 1
    end

   
    del_angle = wrap_to_pi(ang_to_goal - th)
#    @printf "Belief Theta: %d \n" th
    # apply proportional control to compute the turn-rate
    Kprop = 1.0
    om = Kprop * del_angle
#    @printf "Angle to Goal: %d; Delta Angle: %d Heading Change: %d \n" ang_to_goal del_angle om

#    if abs(om) > 2
#	    v = 0.1
#    elseif 1 < abs(om) <= 2
#	    v = 1.0
#    else
#    	v = 2.0
#    end
    v = 5.0
    return RoombaAct(v, om)
end

# first seed the environment
Random.seed!(2)

# reset the policy
p = ToEnd(0) # here, the argument sets the time-steps elapsed to 0
#p = RandomPolicy(m)

# run the simulation
c = @GtkCanvas()
win = GtkWindow(c, "Roomba Environment", 600, 600)
for (t, step) in enumerate(stepthrough(m, p, belief_updater, max_steps=100))
    @guarded draw(c) do widget
#	println(typeof(step))
        #@printf "Time Step: %d; SARS: {} \n" t step#"Time Step: {}; SARS: {}".format(t, step)
        # the following lines render the room, the particles, and the roomba
        ctx = getgc(c)
        set_source_rgb(ctx,1,1,1)
        paint(ctx)
        render(ctx, m, step)
        
        # render some information that can help with debugging
        # here, we render the time-step, the state, and the observation
        move_to(ctx,300,400)
        show_text(ctx, @sprintf("t=%d, state=%s, o=%.3f",t,string(step.s),step.o))
    end
    show(c)
    sleep(0.1) # to slow down the simulation
end

using Statistics

total_rewards = []

for exp = 1:5
    println(string(exp))
    
    Random.seed!(exp)
    
    p = ToEnd(0)
    traj_rewards = sum([step.r for step in stepthrough(m,p,belief_updater, max_steps=100)])
    
    push!(total_rewards, traj_rewards)
end

@printf("Mean Total Reward: %.3f, StdErr Total Reward: %.3f", mean(total_rewards), std(total_rewards)/sqrt(5))
