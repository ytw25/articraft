import { useEffect, useState, type JSX } from "react";

import { useViewer } from "@/lib/viewer-context";
import { fetchRecordFile, fetchRecordTraceFile } from "@/lib/api";
import { findRecordInBootstrap, findRunInBootstrap } from "@/lib/record-summary";
import { TracePanel } from "@/components/inspector/TracePanel";
import { Badge } from "@/components/ui/badge";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Skeleton } from "@/components/ui/skeleton";

function statusVariant(status: string | null | undefined): "success" | "warning" | "destructive" | "secondary" {
  const normalized = (status ?? "").toLowerCase();
  if (normalized === "success" || normalized === "materialized") return "success";
  if (normalized === "running" || normalized === "pending") return "warning";
  if (normalized === "failed") return "destructive";
  return "secondary";
}

function formatDate(value: string | null | undefined): string {
  if (!value) return "--";
  const date = new Date(value);
  if (Number.isNaN(date.getTime())) return value;
  return date.toLocaleString();
}

function Field({ label, children }: { label: string; children: React.ReactNode }): JSX.Element {
  return (
    <div className="bg-white px-2.5 py-[7px]">
      <dt className="text-[10px] font-medium uppercase tracking-[0.04em] text-[#aaa]">{label}</dt>
      <dd className="mt-px whitespace-normal break-words text-[12px] text-[#1e1e1e]">{children}</dd>
    </div>
  );
}

export function MetadataPanel(): JSX.Element {
  const { bootstrap, selectedRecordId } = useViewer();
  const [compileReport, setCompileReport] = useState<Record<string, unknown> | null>(null);
  const [cost, setCost] = useState<Record<string, unknown> | null>(null);
  const [traceText, setTraceText] = useState<string | null>(null);
  const [loadingExtras, setLoadingExtras] = useState(false);

  const record = selectedRecordId ? findRecordInBootstrap(bootstrap, selectedRecordId) : null;
  const run = findRunInBootstrap(bootstrap, record?.run_id);
  const compileStatus =
    typeof compileReport?.status === "string" ? compileReport.status : record?.has_compile_report ? "available" : null;

  useEffect(() => {
    if (!selectedRecordId || !record) {
      setCompileReport(null);
      setCost(null);
      setTraceText(null);
      return;
    }

    let cancelled = false;
    setLoadingExtras(true);

    const promises: Promise<void>[] = [];

    if (record.has_compile_report) {
      promises.push(
        fetchRecordFile(selectedRecordId, "compile_report.json")
          .then((text) => {
            if (!cancelled) setCompileReport(JSON.parse(text) as Record<string, unknown>);
          })
          .catch(() => {
            if (!cancelled) setCompileReport(null);
          }),
      );
    } else {
      setCompileReport(null);
    }

    if (record.has_cost) {
      promises.push(
        fetchRecordFile(selectedRecordId, "cost.json")
          .then((text) => {
            if (!cancelled) setCost(JSON.parse(text) as Record<string, unknown>);
          })
          .catch(() => {
            if (!cancelled) setCost(null);
          }),
      );
    } else {
      setCost(null);
    }

    promises.push(
      fetchRecordTraceFile(selectedRecordId, "conversation.jsonl")
        .then((text) => {
          if (!cancelled) setTraceText(text);
        })
        .catch(() => {
          if (!cancelled) setTraceText(null);
        }),
    );

    Promise.all(promises).finally(() => {
      if (!cancelled) setLoadingExtras(false);
    });

    return () => {
      cancelled = true;
    };
  }, [selectedRecordId, record]);

  if (!selectedRecordId) {
    return (
      <div className="flex h-32 items-center justify-center">
        <p className="text-[11px] text-[#bbb]">Select a record</p>
      </div>
    );
  }

  if (!record) {
    return (
      <div className="flex h-32 items-center justify-center">
        <p className="text-[11px] text-[#bbb]">Record not found</p>
      </div>
    );
  }

  return (
    <ScrollArea className="h-full">
      <div className="space-y-2 pb-1">
        <div>
          <p className="text-[10px] font-medium uppercase tracking-[0.04em] text-[#aaa]">Info</p>
          <p className="mt-1 text-[12px] leading-[1.45] text-[#666]">
            Secondary record diagnostics, provenance hints, and compile artifacts.
          </p>
        </div>

        <div className="grid grid-cols-2 gap-px overflow-hidden rounded-sm border border-[#e8e8e8] bg-[#e8e8e8]">
          <Field label="Record ID">{record.record_id}</Field>
          <Field label="Run ID">{record.run_id ?? "--"}</Field>
          <Field label="Created">{formatDate(record.created_at)}</Field>
          <Field label="Updated">{formatDate(record.updated_at)}</Field>
        </div>

        <div className="flex flex-wrap items-center gap-2">
          <span className="text-[10px] font-medium uppercase tracking-[0.04em] text-[#aaa]">Statuses</span>
          <Badge variant={statusVariant(run?.status)}>{`Run: ${run?.status ?? "unknown"}`}</Badge>
          <Badge variant={statusVariant(compileStatus)}>{`Compile: ${compileStatus ?? "unknown"}`}</Badge>
          <Badge variant={statusVariant(record.materialization_status)}>
            {`Assets: ${record.materialization_status ?? "unknown"}`}
          </Badge>
        </div>

        <div className="grid grid-cols-2 gap-px overflow-hidden rounded-sm border border-[#e8e8e8] bg-[#e8e8e8]">
          <Field label="Collections">{record.collections.join(", ") || "--"}</Field>
          <Field label="Turns">{record.turn_count != null ? String(record.turn_count) : "--"}</Field>
          <Field label="Cost">{record.total_cost_usd != null ? `$${record.total_cost_usd.toFixed(4)}` : "--"}</Field>
          <Field label="Assets">
            {record.materialization_status ?? "unknown"}
          </Field>
        </div>

        {loadingExtras && (
          <div className="space-y-1.5">
            <Skeleton className="h-3 w-24" />
            <Skeleton className="h-14 w-full" />
          </div>
        )}

        {!loadingExtras && compileReport && (
          <div>
            <p className="mb-1.5 text-[10px] font-medium uppercase tracking-[0.04em] text-[#aaa]">Compile Report</p>
            <div className="space-y-px overflow-hidden rounded-sm border border-[#e8e8e8]">
              {Object.entries(compileReport).map(([key, val]) => (
                <div key={key} className="flex flex-col gap-px bg-[#fafafa] px-2.5 py-[6px]">
                  <dt className="font-mono text-[10px] text-[#999]">{key}</dt>
                  <dd className="whitespace-normal break-all font-mono text-[11px] text-[#1e1e1e]">
                    {typeof val === "object" ? JSON.stringify(val) : String(val)}
                  </dd>
                </div>
              ))}
            </div>
          </div>
        )}

        {!loadingExtras && <TracePanel cost={cost} traceText={traceText} />}
      </div>
    </ScrollArea>
  );
}
