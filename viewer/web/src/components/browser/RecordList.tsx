import { useMemo, type JSX } from "react";

import type { RecordSummary } from "@/lib/types";
import { useViewer } from "@/lib/viewer-context";
import { ScrollArea } from "@/components/ui/scroll-area";
import { RecordListItem } from "@/components/browser/RecordListItem";

export function RecordList(): JSX.Element {
  const { bootstrap, searchQuery, sourceFilter, selectedRunId } = useViewer();

  const records = useMemo(() => {
    if (!bootstrap) return [];

    const seen = new Map<string, RecordSummary>();

    const addRecord = (recordId: string, record: RecordSummary | null) => {
      if (!record) return;
      if (!seen.has(recordId)) {
        seen.set(recordId, record);
      }
    };

    if (sourceFilter === "all" || sourceFilter === "workbench") {
      for (const entry of bootstrap.workbench_entries) {
        addRecord(entry.record_id, entry.record);
      }
    }

    if (sourceFilter === "all" || sourceFilter === "dataset") {
      for (const entry of bootstrap.dataset_entries) {
        addRecord(entry.record_id, entry.record);
      }
    }

    if (sourceFilter === "all" || sourceFilter === "runs") {
      for (const entry of bootstrap.workbench_entries) {
        if (entry.record?.run_id) {
          addRecord(entry.record_id, entry.record);
        }
      }
      for (const entry of bootstrap.dataset_entries) {
        if (entry.record?.run_id) {
          addRecord(entry.record_id, entry.record);
        }
      }
    }

    let list = Array.from(seen.values());

    if (selectedRunId) {
      list = list.filter((r) => r.run_id === selectedRunId);
    }

    if (searchQuery.trim()) {
      const query = searchQuery.toLowerCase();
      list = list.filter(
        (r) =>
          r.title.toLowerCase().includes(query) ||
          r.prompt_preview.toLowerCase().includes(query) ||
          r.record_id.toLowerCase().includes(query) ||
          (r.provider && r.provider.toLowerCase().includes(query)) ||
          (r.model_id && r.model_id.toLowerCase().includes(query)),
      );
    }

    list.sort((a, b) => {
      const dateA = a.created_at ? new Date(a.created_at).getTime() : 0;
      const dateB = b.created_at ? new Date(b.created_at).getTime() : 0;
      return dateB - dateA;
    });

    return list;
  }, [bootstrap, searchQuery, sourceFilter, selectedRunId]);

  if (!bootstrap) {
    return (
      <div className="flex flex-1 items-center justify-center p-4">
        <p className="text-[11px] text-[#bbb]">Loading…</p>
      </div>
    );
  }

  if (records.length === 0) {
    return (
      <div className="flex flex-1 items-center justify-center p-4">
        <p className="text-[11px] text-[#bbb]">
          {searchQuery ? "No matching records" : "No records found"}
        </p>
      </div>
    );
  }

  return (
    <ScrollArea className="flex-1">
      <div className="py-0.5">
        {records.map((record) => (
          <RecordListItem key={record.record_id} record={record} />
        ))}
      </div>
    </ScrollArea>
  );
}
