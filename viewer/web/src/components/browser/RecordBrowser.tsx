import { type JSX } from "react";

import type { SourceFilter } from "@/lib/types";
import { useViewer, useViewerDispatch } from "@/lib/viewer-context";
import { RecordSearch } from "@/components/browser/RecordSearch";
import { RunSelector } from "@/components/browser/RunSelector";
import { RecordList } from "@/components/browser/RecordList";

const filters: { value: SourceFilter; label: string }[] = [
  { value: "all", label: "All" },
  { value: "runs", label: "Runs" },
  { value: "workbench", label: "Workbench" },
  { value: "dataset", label: "Dataset" },
];

export function RecordBrowser(): JSX.Element {
  const { sourceFilter, loading, error } = useViewer();
  const dispatch = useViewerDispatch();

  const showRunSelector = sourceFilter === "all" || sourceFilter === "runs";

  return (
    <div className="flex h-full min-w-0 flex-col">
      {/* Section label */}
      <div className="px-3 pb-0 pt-3">
        <span className="text-[11px] font-medium uppercase tracking-[0.06em] text-[#999]">
          Explorer
        </span>
      </div>

      {/* Filter tabs — minimal underline style */}
      <div className="flex gap-0 border-b border-[#e8e8e8] px-3">
        {filters.map((f) => (
          <button
            key={f.value}
            type="button"
            onClick={() =>
              dispatch({ type: "SET_SOURCE_FILTER", payload: f.value })
            }
            className={`border-b-[1.5px] px-2.5 pb-2 pt-2.5 text-[11px] font-medium transition-colors ${
              sourceFilter === f.value
                ? "border-[#1e1e1e] text-[#1e1e1e]"
                : "border-transparent text-[#999] hover:text-[#666]"
            }`}
          >
            {f.label}
          </button>
        ))}
      </div>

      {/* Search + run filter */}
      <div className="space-y-2 px-2.5 py-2.5">
        <RecordSearch />
        {showRunSelector ? <RunSelector /> : null}
      </div>

      {error ? (
        <div className="border-b border-red-100 bg-red-50/50 px-3 py-2">
          <p className="text-[11px] text-red-600">{error}</p>
        </div>
      ) : null}

      {loading ? (
        <div className="flex flex-1 items-center justify-center">
          <p className="text-[11px] text-[#bbb]">Loading…</p>
        </div>
      ) : (
        <RecordList />
      )}
    </div>
  );
}
