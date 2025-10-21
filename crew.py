#!/usr/bin/env python3
"""
AI Mower Crew - Autonomous Lawn Mower Development Crew
A CrewAI-based multi-agent system for designing and developing a ROS2-based autonomous lawn mower.
"""

from crewai import Agent, Crew, Process, Task
from crewai.project import CrewBase, agent, crew, task
from pathlib import Path
import yaml


@CrewBase
class AiMowerCrew:
    """AI Mower Crew - Multi-agent system for autonomous lawn mower development"""

    agents_config = 'config/agents.yaml'
    tasks_config = 'config/tasks.yaml'

    def __init__(self):
        """Initialize the crew with configuration files"""
        self.agents_config_path = Path(__file__).parent / self.agents_config
        self.tasks_config_path = Path(__file__).parent / self.tasks_config

    # ==================== AGENTS ====================

    @agent
    def system_architect(self) -> Agent:
        return Agent(
            config=self.agents_config['system_architect'],
            verbose=True
        )

    @agent
    def safety_engineer(self) -> Agent:
        return Agent(
            config=self.agents_config['safety_engineer'],
            verbose=True
        )

    @agent
    def navigation_specialist(self) -> Agent:
        return Agent(
            config=self.agents_config['navigation_specialist'],
            verbose=True
        )

    @agent
    def differential_drive_specialist(self) -> Agent:
        return Agent(
            config=self.agents_config['differential_drive_specialist'],
            verbose=True
        )

    @agent
    def ros_code_hunter(self) -> Agent:
        return Agent(
            config=self.agents_config['ros_code_hunter'],
            verbose=True
        )

    @agent
    def simulator(self) -> Agent:
        return Agent(
            config=self.agents_config['simulator'],
            verbose=True
        )

    @agent
    def test_specialist(self) -> Agent:
        return Agent(
            config=self.agents_config['test_specialist'],
            verbose=True
        )

    @agent
    def realist(self) -> Agent:
        return Agent(
            config=self.agents_config['realist'],
            verbose=True
        )

    @agent
    def ros_infrastructure_builder(self) -> Agent:
        return Agent(
            config=self.agents_config['ros_infrastructure_builder'],
            verbose=True
        )

    @agent
    def controller_plugin_integrator(self) -> Agent:
        return Agent(
            config=self.agents_config['controller_plugin_integrator'],
            verbose=True
        )

    # ==================== TASKS ====================

    @task
    def architecture_design(self) -> Task:
        return Task(
            config=self.tasks_config['architecture_design'],
        )

    @task
    def safety_analysis(self) -> Task:
        return Task(
            config=self.tasks_config['safety_analysis'],
        )

    @task
    def navigation_stack_selection(self) -> Task:
        return Task(
            config=self.tasks_config['navigation_stack_selection'],
        )

    @task
    def differential_drive_design(self) -> Task:
        return Task(
            config=self.tasks_config['differential_drive_design'],
        )

    @task
    def package_search(self) -> Task:
        return Task(
            config=self.tasks_config['package_search'],
        )

    @task
    def urdf_design(self) -> Task:
        return Task(
            config=self.tasks_config['urdf_design'],
        )

    @task
    def gazebo_world_design(self) -> Task:
        return Task(
            config=self.tasks_config['gazebo_world_design'],
        )

    @task
    def component_test_protocol(self) -> Task:
        return Task(
            config=self.tasks_config['component_test_protocol'],
        )

    @task
    def feasibility_review(self) -> Task:
        return Task(
            config=self.tasks_config['feasibility_review'],
        )

    @task
    def launch_system_design(self) -> Task:
        return Task(
            config=self.tasks_config['launch_system_design'],
        )

    @task
    def controller_integration_plan(self) -> Task:
        return Task(
            config=self.tasks_config['controller_integration_plan'],
        )

    @task
    def knowledge_base_creation(self) -> Task:
        return Task(
            config=self.tasks_config['knowledge_base_creation'],
        )

    # ==================== CREW ====================

    @crew
    def crew(self) -> Crew:
        """Creates the AI Mower Crew with all agents and tasks"""
        return Crew(
            agents=self.agents,  # Automatically includes all @agent decorated methods
            tasks=self.tasks,    # Automatically includes all @task decorated methods
            process=Process.sequential,  # Tasks will be executed sequentially
            verbose=True,
        )
