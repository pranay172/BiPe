"""Run a trusted project CAD script through the local MCP stdio server."""

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
    code += (
        "exec(compile("
        + repr(args.script.read_text())
        + ", "
        + repr(str(args.script))
        + ", 'exec'))"
    )
    logpath = ROOT / args.log
    logpath.parent.mkdir(parents=True, exist_ok=True)
    with logpath.open("w") as log:
        async with (
            stdio_client(
                StdioServerParameters(command=str(ROOT / "tools/freecad-mcp.sh"), cwd=ROOT),
                errlog=log,
            ) as streams,
            ClientSession(
                *streams, read_timeout_seconds=timedelta(seconds=args.timeout + 20)
            ) as session,
        ):
            await session.initialize()
            result = await session.call_tool(
                "execute_python", {"code": code, "timeout_ms": args.timeout * 1000}
            )
            data = result.structuredContent
            if data is None:
                data = json.loads(next(c.text for c in result.content if c.type == "text"))
            print(json.dumps(data, indent=2))
            if result.isError or not data.get("success"):
                raise RuntimeError("CAD execution failed; see returned error and saved MCP log")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("script", type=Path)
    parser.add_argument("--timeout", type=int, default=45)
    parser.add_argument("--log", default="artifacts/tooling/freecad-mcp/call.log")
    asyncio.run(call(parser.parse_args()))
