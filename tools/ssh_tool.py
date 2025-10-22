#!/usr/bin/env python3
"""
SSH Tool for AI Mower Crew
Allows agents to execute commands on remote systems via SSH
"""

from crewai.tools import BaseTool
from typing import Type, Optional
from pydantic import BaseModel, Field
import paramiko
import time


class SSHCommandInput(BaseModel):
    """Input schema for SSH command execution."""
    host: str = Field(..., description="Hostname or IP address of the remote system")
    username: str = Field(..., description="SSH username")
    password: str = Field(..., description="SSH password")
    command: str = Field(..., description="Command to execute on the remote system")
    timeout: int = Field(default=60, description="Command timeout in seconds")
    sudo: bool = Field(default=False, description="Run command with sudo")


class SSHTool(BaseTool):
    name: str = "SSH Command Executor"
    description: str = (
        "Executes commands on a remote system via SSH. "
        "Use this to configure BOB (Raspberry Pi) remotely. "
        "Supports standard commands and sudo execution. "
        "Returns command output (stdout and stderr)."
    )
    args_schema: Type[BaseModel] = SSHCommandInput

    def _run(
        self,
        host: str,
        username: str,
        password: str,
        command: str,
        timeout: int = 60,
        sudo: bool = False
    ) -> str:
        """
        Execute a command via SSH on a remote system.

        Args:
            host: Remote hostname or IP address
            username: SSH username
            password: SSH password
            command: Command to execute
            timeout: Command timeout in seconds
            sudo: Whether to run with sudo

        Returns:
            Command output as a string
        """
        try:
            # Create SSH client
            client = paramiko.SSHClient()
            client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

            # Connect to remote host
            client.connect(
                hostname=host,
                username=username,
                password=password,
                timeout=10
            )

            # Prepare command
            if sudo:
                # Use -S to read password from stdin
                full_command = f'echo "{password}" | sudo -S {command}'
            else:
                full_command = command

            # Execute command
            stdin, stdout, stderr = client.exec_command(
                full_command,
                timeout=timeout,
                get_pty=True  # Required for sudo
            )

            # Get output
            output = stdout.read().decode('utf-8')
            error = stderr.read().decode('utf-8')
            exit_code = stdout.channel.recv_exit_status()

            # Close connection
            client.close()

            # Format result
            result = f"Exit Code: {exit_code}\n\n"
            if output:
                result += f"STDOUT:\n{output}\n"
            if error:
                result += f"STDERR:\n{error}\n"

            return result

        except paramiko.AuthenticationException:
            return f"ERROR: Authentication failed for {username}@{host}"
        except paramiko.SSHException as e:
            return f"ERROR: SSH connection failed: {str(e)}"
        except Exception as e:
            return f"ERROR: Command execution failed: {str(e)}"


class SSHFileTransferInput(BaseModel):
    """Input schema for SSH file transfer."""
    host: str = Field(..., description="Hostname or IP address")
    username: str = Field(..., description="SSH username")
    password: str = Field(..., description="SSH password")
    local_path: str = Field(..., description="Local file path")
    remote_path: str = Field(..., description="Remote file path")
    direction: str = Field(..., description="'upload' or 'download'")


class SSHFileTransferTool(BaseTool):
    name: str = "SSH File Transfer"
    description: str = (
        "Transfers files to/from remote system via SFTP. "
        "Use direction='upload' to send files to BOB, "
        "or direction='download' to retrieve files from BOB."
    )
    args_schema: Type[BaseModel] = SSHFileTransferInput

    def _run(
        self,
        host: str,
        username: str,
        password: str,
        local_path: str,
        remote_path: str,
        direction: str
    ) -> str:
        """
        Transfer files via SFTP.

        Args:
            host: Remote hostname or IP
            username: SSH username
            password: SSH password
            local_path: Local file path
            remote_path: Remote file path
            direction: 'upload' or 'download'

        Returns:
            Status message
        """
        try:
            # Create SSH client
            transport = paramiko.Transport((host, 22))
            transport.connect(username=username, password=password)

            # Create SFTP client
            sftp = paramiko.SFTPClient.from_transport(transport)

            # Transfer file
            if direction == 'upload':
                sftp.put(local_path, remote_path)
                result = f"Successfully uploaded {local_path} to {remote_path} on {host}"
            elif direction == 'download':
                sftp.get(remote_path, local_path)
                result = f"Successfully downloaded {remote_path} from {host} to {local_path}"
            else:
                result = f"ERROR: Invalid direction '{direction}'. Use 'upload' or 'download'."

            # Close connections
            sftp.close()
            transport.close()

            return result

        except FileNotFoundError as e:
            return f"ERROR: File not found: {str(e)}"
        except Exception as e:
            return f"ERROR: File transfer failed: {str(e)}"


# Tool instances for CrewAI
ssh_command_tool = SSHTool()
ssh_file_transfer_tool = SSHFileTransferTool()
