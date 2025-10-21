#!/usr/bin/env python3
"""
GitHub Search Tool for finding ROS2 packages and robotics code
Now with actual GitHub API integration!
"""

from crewai_tools import BaseTool
from typing import Optional, Type, List, Dict
from pydantic import BaseModel, Field
import requests
import time

# Import GitHub credentials
try:
    from .github_config import GITHUB_TOKEN, get_headers
    HAS_CREDENTIALS = True
except ImportError:
    HAS_CREDENTIALS = False
    print("Warning: GitHub credentials not found. Using unauthenticated API (rate limited).")


class GitHubSearchToolInput(BaseModel):
    """Input schema for GitHub Search Tool"""
    query: str = Field(..., description="Search query for GitHub repositories")
    language: Optional[str] = Field(default=None, description="Programming language filter (e.g., 'Python', 'C++')")
    topic: Optional[str] = Field(default=None, description="GitHub topic filter (e.g., 'ros2', 'robotics')")
    max_results: Optional[int] = Field(default=10, description="Maximum number of results to return")


class GitHubSearchTool(BaseTool):
    name: str = "GitHub ROS2 Package Search"
    description: str = (
        "Searches GitHub for ROS2 packages, robotics libraries, and related code. "
        "Useful for finding existing implementations of sensors, controllers, navigation, "
        "SLAM, and other robotics functionality. Returns repository names, URLs, descriptions, "
        "star counts, and maintenance status."
    )
    args_schema: Type[BaseModel] = GitHubSearchToolInput

    def _run(
        self,
        query: str,
        language: Optional[str] = None,
        topic: Optional[str] = None,
        max_results: int = 10
    ) -> str:
        """
        Execute the GitHub search using GitHub API

        Args:
            query: Search query
            language: Programming language filter
            topic: Topic filter
            max_results: Maximum results to return

        Returns:
            Formatted string with search results
        """

        # Build search query
        search_terms = [query]

        if language:
            search_terms.append(f"language:{language}")
        if topic:
            search_terms.append(f"topic:{topic}")

        # Add ROS2 context if not already in query
        if "ros2" not in query.lower() and "ros" not in query.lower():
            search_terms.append("ros2")

        full_query = " ".join(search_terms)

        # Prepare API request
        api_url = "https://api.github.com/search/repositories"
        params = {
            "q": full_query,
            "sort": "stars",
            "order": "desc",
            "per_page": min(max_results, 30)  # GitHub API max is 30 per page
        }

        # Set headers
        if HAS_CREDENTIALS:
            headers = get_headers()
        else:
            headers = {"Accept": "application/vnd.github.v3+json"}

        try:
            # Make API request
            response = requests.get(api_url, params=params, headers=headers, timeout=10)

            # Check rate limit
            if response.status_code == 403:
                return self._format_rate_limit_error(response)

            response.raise_for_status()
            data = response.json()

            # Format results
            return self._format_results(data, full_query, max_results)

        except requests.exceptions.Timeout:
            return f"Error: GitHub API request timed out for query: {full_query}"
        except requests.exceptions.RequestException as e:
            return f"Error searching GitHub: {str(e)}\n\nTry manual search at: https://github.com/search?q={query.replace(' ', '+')}"

    def _format_results(self, data: Dict, query: str, max_results: int) -> str:
        """Format GitHub API results into readable output"""

        total_count = data.get("total_count", 0)
        items = data.get("items", [])

        if total_count == 0:
            return f"No GitHub repositories found for query: {query}"

        # Build formatted output
        output = []
        output.append("=" * 80)
        output.append(f"GitHub Search Results for: {query}")
        output.append(f"Total repositories found: {total_count}")
        output.append(f"Showing top {min(len(items), max_results)} results")
        output.append("=" * 80)
        output.append("")

        for idx, repo in enumerate(items[:max_results], 1):
            output.append(f"{idx}. {repo['full_name']}")
            output.append(f"   ⭐ Stars: {repo['stargazers_count']:,}")
            output.append(f"   🔗 URL: {repo['html_url']}")

            if repo.get('description'):
                output.append(f"   📝 Description: {repo['description'][:150]}")

            if repo.get('language'):
                output.append(f"   💻 Language: {repo['language']}")

            # Additional useful info
            output.append(f"   📅 Updated: {repo['updated_at'][:10]}")
            output.append(f"   🍴 Forks: {repo['forks_count']}")
            output.append(f"   ⚠️  Issues: {repo['open_issues_count']}")

            # Check for topics (ROS2 distro, etc.)
            if repo.get('topics'):
                topics = [t for t in repo['topics'] if 'ros' in t or 'jazzy' in t or 'humble' in t]
                if topics:
                    output.append(f"   🏷️  Topics: {', '.join(topics)}")

            output.append("")

        output.append("-" * 80)
        output.append("Use these repositories as starting points for your autonomous mower project!")
        output.append("-" * 80)

        return "\n".join(output)

    def _format_rate_limit_error(self, response) -> str:
        """Format rate limit error with helpful information"""
        reset_time = response.headers.get('X-RateLimit-Reset', 'unknown')

        if reset_time != 'unknown':
            import datetime
            reset_dt = datetime.datetime.fromtimestamp(int(reset_time))
            reset_str = reset_dt.strftime('%Y-%m-%d %H:%M:%S')
        else:
            reset_str = 'unknown'

        return f"""
GitHub API Rate Limit Exceeded

Your rate limit will reset at: {reset_str}

Options:
1. Wait for rate limit to reset
2. Use authenticated requests (configure github_config.py with your token)
3. Search manually at: https://github.com/search

Current authentication status: {'Authenticated' if HAS_CREDENTIALS else 'Unauthenticated'}
Rate limits:
  - Unauthenticated: 60 requests per hour
  - Authenticated: 5,000 requests per hour
"""


# Test/demo function
if __name__ == "__main__":
    tool = GitHubSearchTool()

    print("Testing GitHub Search Tool...\n")

    # Test search
    result = tool._run(
        query="differential drive controller",
        language="Python",
        topic="ros2",
        max_results=5
    )

    print(result)

    print("\n" + "=" * 80)
    print("Additional test: SLAM packages")
    print("=" * 80 + "\n")

    result2 = tool._run(
        query="slam toolbox",
        topic="ros2",
        max_results=3
    )

    print(result2)
