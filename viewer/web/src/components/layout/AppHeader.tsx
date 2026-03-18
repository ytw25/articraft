import { type JSX } from "react";
import { RefreshCw } from "lucide-react";

import { fetchBootstrap } from "@/lib/api";
import { useViewer, useViewerDispatch } from "@/lib/viewer-context";
import { Button } from "@/components/ui/button";

const TRAILING_PUNCTUATION = /[\s,.;:!?)]*$/;

function truncateWithEllipsis(value: string, maxLength = 88): string {
  const normalized = value.replace(/\s+/g, " ").trim();
  if (!normalized) return "";

  const withoutExistingEllipsis = normalized.replace(/\.\.\.$/, "").trimEnd();
  if (withoutExistingEllipsis.length <= maxLength) {
    return withoutExistingEllipsis;
  }

  const truncated = withoutExistingEllipsis.slice(0, maxLength).trimEnd();
  return `${truncated.replace(TRAILING_PUNCTUATION, "")}...`;
}

export function AppHeader(): JSX.Element {
  const state = useViewer();
  const dispatch = useViewerDispatch();

  const selectedRecord = (() => {
    if (!state.bootstrap || !state.selectedRecordId) return null;

    for (const entry of state.bootstrap.workbench_entries) {
      if (entry.record_id === state.selectedRecordId && entry.record) {
        return entry.record;
      }
    }
    for (const entry of state.bootstrap.dataset_entries) {
      if (entry.record_id === state.selectedRecordId && entry.record) {
        return entry.record;
      }
    }
    return null;
  })();

  const selectedRecordTitleFull = selectedRecord?.title ?? null;
  const selectedRecordTitle = selectedRecord ? truncateWithEllipsis(selectedRecord.title, 72) : null;

  const handleRefresh = () => {
    dispatch({ type: "SET_LOADING", payload: true });
    fetchBootstrap()
      .then((data) => {
        dispatch({ type: "SET_BOOTSTRAP", payload: data });
      })
      .catch((err) => {
        dispatch({
          type: "SET_ERROR",
          payload: err instanceof Error ? err.message : "Failed to refresh.",
        });
      });
  };

  return (
    <header className="flex h-10 shrink-0 items-center gap-3 border-b border-[#e4e4e4] bg-[#f8f8f8] px-3">
      <div className="flex items-center gap-1.5 text-[12px]">
        <span className="font-medium text-[#1e1e1e]">Articraft</span>
        <span className="text-[#ccc]">/</span>
        <span className="text-[#888]">Viewer</span>
      </div>

      <div className="mx-1 flex min-w-0 flex-1 items-center justify-center">
        {selectedRecordTitle ? (
          <p
            className="max-w-full truncate text-[12px] text-[#666]"
            title={selectedRecordTitleFull ?? undefined}
          >
            {selectedRecordTitle}
          </p>
        ) : (
          <p className="text-[12px] text-[#bbb]">No record selected</p>
        )}
      </div>

      <div className="flex items-center">
        <Button
          variant="ghost"
          size="sm"
          onClick={handleRefresh}
          disabled={state.loading}
          className="h-7 w-7 p-0 text-[#999] hover:bg-[#eee] hover:text-[#666]"
        >
          <RefreshCw
            className={`size-3.5 ${state.loading ? "animate-spin" : ""}`}
          />
        </Button>
      </div>
    </header>
  );
}
