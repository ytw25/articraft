import { type JSX } from "react";

import type { RecordSummary } from "@/lib/types";
import { useViewer, useViewerDispatch } from "@/lib/viewer-context";

interface RecordListItemProps {
  record: RecordSummary;
}

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

function formatCost(value: number | null): string | null {
  if (value === null || Number.isNaN(value)) return null;
  if (value < 0.01) return `$${value.toFixed(3)}`;
  if (value < 0.1) return `$${value.toFixed(3)}`;
  return new Intl.NumberFormat(undefined, {
    style: "currency",
    currency: "USD",
    minimumFractionDigits: 2,
    maximumFractionDigits: 2,
  }).format(value);
}

function formatDate(value: string | null): string | null {
  if (!value) return null;
  const date = new Date(value);
  if (Number.isNaN(date.getTime())) return null;

  const now = new Date();
  const sameYear = date.getFullYear() === now.getFullYear();
  return new Intl.DateTimeFormat(undefined, {
    month: "short",
    day: "numeric",
    ...(sameYear ? {} : { year: "numeric" }),
  }).format(date);
}

export function RecordListItem({ record }: RecordListItemProps): JSX.Element {
  const { selectedRecordId } = useViewer();
  const dispatch = useViewerDispatch();
  const isSelected = selectedRecordId === record.record_id;
  const summaryText = truncateWithEllipsis(record.prompt_preview || record.title);
  const metadata = [
    record.model_id,
    record.turn_count !== null
      ? `${record.turn_count} turn${record.turn_count === 1 ? "" : "s"}`
      : null,
    formatCost(record.total_cost_usd),
    formatDate(record.updated_at ?? record.created_at),
  ].filter((item): item is string => Boolean(item));

  return (
    <button
      type="button"
      onClick={() =>
        dispatch({ type: "SELECT_RECORD", payload: record.record_id })
      }
      className={`w-full px-3 py-2 text-left transition-colors ${
        isSelected
          ? "bg-[#007acc]/[0.08]"
          : "hover:bg-[#e8e8e8]/60"
      }`}
      title={summaryText}
    >
      <p
        className={`break-words text-[11px] leading-[1.4] ${
          isSelected ? "font-medium text-[#1e1e1e]" : "text-[#3f3f3f]"
        }`}
      >
        {summaryText}
      </p>

      {metadata.length > 0 ? (
        <div className="mt-1 flex flex-wrap items-center gap-x-1.5 gap-y-0.5 text-[9px] text-[#7e7e7e]">
          {metadata.map((item, index) => (
            <span key={`${record.record_id}-${item}`} className="flex items-center gap-x-1.5">
              {index > 0 ? <span className="text-[#b3b3b3]">·</span> : null}
              <span>{item}</span>
            </span>
          ))}
        </div>
      ) : null}
    </button>
  );
}
