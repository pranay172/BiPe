"""Execute a trusted project script through the official Blender MCP stdio server."""

import argparse
import asyncio
import json
from datetime import timedelta
from pathlib import Path

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

ROOT = Path(__file__).resolve().parents[1]


async def call(args):
    code = "PROJECT_ROOT = " + repr(str(ROOT)) + "\n"
    code += "exec(compile(" + repr(args.script.read_text()) + ", " + repr(str(args.script))
    code += ", 'exec'))"
    logpath = ROOT / args.log
    logpath.parent.mkdir(parents=True, exist_ok=True)
    with logpath.open("w") as log:
        async with (
            stdio_client(
                StdioServerParameters(command=str(ROOT / "tools/blender-mcp.sh"), cwd=ROOT),
                errlog=log,
            ) as streams,
            ClientSession(
                *streams, read_timeout_seconds=timedelta(seconds=args.timeout)
            ) as session,
        ):
            init = await session.initialize()
            listing = await session.list_tools()
            result = await session.call_tool("execute_blender_code", {"code": code})
            if result.isError:
                print(result.model_dump_json(indent=2))
                raise RuntimeError("Blender MCP tool returned an error")
            data = result.structuredContent
            if data is None:
                data = json.loads(next(c.text for c in result.content if c.type == "text"))
            print(
                json.dumps(
                    {
                        "server": init.serverInfo.model_dump(),
                        "tools": len(listing.tools),
                        "response": data,
                    },
                    indent=2,
                )
            )
            if isinstance(data, dict) and (data.get("error") or data.get("status") == "error"):
                raise RuntimeError("Blender execution failed; see response and MCP log")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("script", type=Path)
    parser.add_argument("--timeout", type=int, default=60)
    parser.add_argument("--log", default="artifacts/tooling/blender-mcp/stdio.log")
    asyncio.run(call(parser.parse_args()))
