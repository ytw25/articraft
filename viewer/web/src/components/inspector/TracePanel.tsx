import { useEffect, useState, type JSX } from "react";
import { ChevronDown } from "lucide-react";

import { Badge } from "@/components/ui/badge";

type TracePanelProps = {
  cost: Record<string, unknown> | null;
  traceText: string | null;
};

type TraceMessage = {
  role: string;
  content?: unknown;
  tool_calls?: Array<{ function?: { name?: string } }>;
  name?: string;
};

type TraceTurn = {
  index: number;
  cost: Record<string, unknown> | null;
  events: TraceMessage[];
};

function asRecord(value: unknown): Record<string, unknown> | null {
  return typeof value === "object" && value !== null && !Array.isArray(value)
    ? (value as Record<string, unknown>)
    : null;
}

function asNumber(value: unknown): number | null {
  return typeof value === "number" ? value : null;
}

function formatUsd(value: unknown): string {
  const number = asNumber(value);
  return number == null ? "--" : `$${number.toFixed(4)}`;
}

function formatCount(value: unknown): string {
  const number = asNumber(value);
  return number == null ? "--" : number.toLocaleString();
}

function excerpt(text: string, max = 220): string {
  const normalized = text.replace(/\s+/g, " ").trim();
  if (normalized.length <= max) return normalized;
  return `${normalized.slice(0, max - 1)}...`;
}

function messageText(content: unknown): string {
  if (typeof content === "string") return content;
  if (Array.isArray(content)) {
    return content
      .map((item) => {
        if (typeof item === "string") return item;
        const record = asRecord(item);
        if (!record) return "";
        if (typeof record.text === "string") return record.text;
        return "";
      })
      .filter(Boolean)
      .join("\n");
  }
  if (content == null) return "";
  try {
    return JSON.stringify(content);
  } catch {
    return String(content);
  }
}

function summarizeToolContent(content: unknown): string {
  const raw = messageText(content);
  if (!raw) return "";

  try {
    const parsed = JSON.parse(raw) as Record<string, unknown>;
    const result = typeof parsed.result === "string" ? parsed.result : null;
    const compilation = asRecord(parsed.compilation);
    const compilationStatus = typeof compilation?.status === "string" ? compilation.status : null;
    const parts = [result, compilationStatus ? `compilation ${compilationStatus}` : null].filter(Boolean);
    if (parts.length > 0) return parts.join(", ");
  } catch {
    // Fall back to plain text excerpt.
  }

  return excerpt(raw, 180);
}

function fullMessageText(content: unknown): string {
  if (typeof content === "string") return content;
  if (Array.isArray(content)) {
    const parts = content
      .map((item) => {
        if (typeof item === "string") return item;
        const record = asRecord(item);
        if (!record) return null;
        if (typeof record.text === "string") return record.text;
        try {
          return JSON.stringify(record, null, 2);
        } catch {
          return String(record);
        }
      })
      .filter((value): value is string => Boolean(value));

    return parts.join("\n\n");
  }
  if (content == null) return "";
  if (typeof content === "object") {
    try {
      return JSON.stringify(content, null, 2);
    } catch {
      return String(content);
    }
  }
  return String(content);
}

function messageVariant(role: string): "default" | "secondary" | "warning" {
  if (role === "assistant") return "default";
  if (role === "tool") return "warning";
  return "secondary";
}

function parseTurns(traceText: string | null, cost: Record<string, unknown> | null): TraceTurn[] {
  const costTurns = Array.isArray(cost?.turns) ? cost.turns : [];
  if (!traceText) {
    return costTurns.map((turn, index) => ({
      index: index + 1,
      cost: asRecord(turn),
      events: [],
    }));
  }

  const entries = traceText
    .split(/\r?\n/)
    .map((line) => line.trim())
    .filter(Boolean)
    .map((line) => {
      try {
        return JSON.parse(line) as Record<string, unknown>;
      } catch {
        return null;
      }
    })
    .filter((entry): entry is Record<string, unknown> => entry !== null);

  const turns: TraceTurn[] = [];

  for (const entry of entries) {
    const message = asRecord(entry.message);
    const role = typeof message?.role === "string" ? message.role : null;
    if (!message || !role) continue;

    const traceMessage: TraceMessage = {
      role,
      content: message.content,
      tool_calls: Array.isArray(message.tool_calls)
        ? (message.tool_calls as Array<{ function?: { name?: string } }>)
        : undefined,
      name: typeof message.name === "string" ? message.name : undefined,
    };

    if (role === "assistant") {
      turns.push({
        index: turns.length + 1,
        cost: asRecord(costTurns[turns.length]),
        events: [traceMessage],
      });
      continue;
    }

    if (turns.length === 0) continue;
    turns[turns.length - 1].events.push(traceMessage);
  }

  if (turns.length === 0) {
    return costTurns.map((turn, index) => ({
      index: index + 1,
      cost: asRecord(turn),
      events: [],
    }));
  }

  return turns;
}

function renderEventTitle(event: TraceMessage): string {
  if (event.role === "assistant") {
    const toolCalls = event.tool_calls ?? [];
    if (toolCalls.length > 0) {
      const names = toolCalls
        .map((call) => call.function?.name)
        .filter((name): name is string => Boolean(name));
      return names.length > 0 ? `Called ${names.join(", ")}` : "Assistant step";
    }
    return "Assistant response";
  }
  if (event.role === "tool") {
    return event.name ?? "Tool result";
  }
  return "Feedback";
}

function renderEventBody(event: TraceMessage): string {
  return fullMessageText(event.content);
}

function turnSummary(turn: TraceTurn): string {
  const assistantEvent = turn.events.find((event) => event.role === "assistant");
  if (assistantEvent) {
    const toolCalls = assistantEvent.tool_calls ?? [];
    if (toolCalls.length > 0) {
      const names = toolCalls
        .map((call) => call.function?.name)
        .filter((name): name is string => Boolean(name));
      if (names.length > 0) return `Calls ${names.join(", ")}`;
    }

    const text = messageText(assistantEvent.content);
    if (text) return excerpt(text, 96);
  }

  const toolEvent = turn.events.find((event) => event.role === "tool");
  if (toolEvent) {
    return summarizeToolContent(toolEvent.content) || (toolEvent.name ?? "Tool activity");
  }

  return "No trace content";
}

export function TracePanel({ cost, traceText }: TracePanelProps): JSX.Element | null {
  const total = asRecord(cost?.total);
  const totalTokens = asRecord(total?.tokens);
  const totalCosts = asRecord(total?.costs_usd);
  const turns = parseTurns(traceText, cost);
  const modelId = typeof cost?.model_id === "string" ? cost.model_id : null;
  const [openTurns, setOpenTurns] = useState<Set<number>>(() => new Set(turns[0] ? [turns[0].index] : []));

  useEffect(() => {
    setOpenTurns(new Set(turns[0] ? [turns[0].index] : []));
  }, [traceText, cost]);

  if (!cost && !traceText) return null;

  const toggleTurn = (turnIndex: number): void => {
    setOpenTurns((prev) => {
      const next = new Set(prev);
      if (next.has(turnIndex)) {
        next.delete(turnIndex);
      } else {
        next.add(turnIndex);
      }
      return next;
    });
  };

  return (
    <div className="space-y-2.5">
      <div>
        <p className="mb-1.5 text-[10px] font-medium uppercase tracking-[0.04em] text-[#aaa]">Agent Trace</p>
        <div className="grid grid-cols-2 gap-px overflow-hidden rounded-sm border border-[#e8e8e8] bg-[#e8e8e8]">
          <div className="bg-white px-2.5 py-[7px]">
            <dt className="text-[10px] font-medium uppercase tracking-[0.04em] text-[#aaa]">Total Cost</dt>
            <dd className="mt-px text-[12px] text-[#1e1e1e]">{formatUsd(totalCosts?.total)}</dd>
          </div>
          <div className="bg-white px-2.5 py-[7px]">
            <dt className="text-[10px] font-medium uppercase tracking-[0.04em] text-[#aaa]">Total Tokens</dt>
            <dd className="mt-px text-[12px] text-[#1e1e1e]">{formatCount(totalTokens?.total_tokens)}</dd>
          </div>
          <div className="bg-white px-2.5 py-[7px]">
            <dt className="text-[10px] font-medium uppercase tracking-[0.04em] text-[#aaa]">Turns</dt>
            <dd className="mt-px text-[12px] text-[#1e1e1e]">{turns.length}</dd>
          </div>
          <div className="bg-white px-2.5 py-[7px]">
            <dt className="text-[10px] font-medium uppercase tracking-[0.04em] text-[#aaa]">Model</dt>
            <dd className="mt-px break-all text-[12px] text-[#1e1e1e]">{modelId ?? "--"}</dd>
          </div>
        </div>
      </div>

      <div className="space-y-2">
        {turns.map((turn) => {
          const turnTokens = asRecord(turn.cost?.tokens);
          const turnCosts = asRecord(turn.cost?.costs_usd);
          const isOpen = openTurns.has(turn.index);

          return (
            <div key={turn.index} className="overflow-hidden rounded-sm border border-[#e8e8e8] bg-white">
              <button
                type="button"
                onClick={() => toggleTurn(turn.index)}
                className="flex w-full items-start justify-between gap-3 bg-[#fafafa] px-2.5 py-2 text-left transition-colors hover:bg-[#f6f6f6]"
                aria-expanded={isOpen}
              >
                <div className="min-w-0 flex-1">
                  <div className="flex flex-wrap items-center gap-x-2 gap-y-1">
                    <p className="font-mono text-[11px] text-[#1e1e1e]">Turn {turn.index}</p>
                    <p className="text-[10px] text-[#999]">
                      {turn.events.length > 0 ? `${turn.events.length} event${turn.events.length === 1 ? "" : "s"}` : "No events"}
                    </p>
                  </div>
                  <p className="mt-1 pr-2 text-[11px] leading-[1.45] text-[#666]">
                    {turnSummary(turn)}
                  </p>
                </div>
                <div className="flex shrink-0 items-start gap-1.5">
                  <Badge variant="success">{formatUsd(turnCosts?.total)}</Badge>
                  <Badge variant="secondary">{`${formatCount(turnTokens?.total_tokens)} tok`}</Badge>
                  <ChevronDown
                    className={`mt-[1px] size-3.5 text-[#999] transition-transform ${isOpen ? "rotate-180" : ""}`}
                  />
                </div>
              </button>

              {isOpen ? (
                <div className="space-y-2 border-t border-[#efefef] px-2.5 py-2.5">
                  {turn.events.length > 0 ? (
                    turn.events.map((event, eventIndex) => {
                      const body = renderEventBody(event);
                      return (
                        <div key={`${turn.index}-${eventIndex}`} className="rounded-sm border border-[#ededed] bg-[#fcfcfc] px-2.5 py-2">
                          <div className="flex flex-wrap items-center gap-2">
                            <Badge variant={messageVariant(event.role)}>{event.role}</Badge>
                            <p className="text-[11px] text-[#1e1e1e]">{renderEventTitle(event)}</p>
                          </div>
                          {body ? (
                            <pre className="mt-1.5 overflow-x-auto whitespace-pre-wrap break-words font-mono text-[10px] leading-[1.55] text-[#666]">
                              {body}
                            </pre>
                          ) : null}
                        </div>
                      );
                    })
                  ) : (
                    <div className="rounded-sm border border-dashed border-[#e4e4e4] bg-[#fcfcfc] px-2.5 py-2 text-[10px] text-[#999]">
                      Trace unavailable for this turn.
                    </div>
                  )}
                </div>
              ) : null}
            </div>
          );
        })}
      </div>
    </div>
  );
}
