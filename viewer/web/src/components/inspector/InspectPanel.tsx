import { useEffect, useMemo, useState, type JSX } from "react";
import { RotateCcw } from "lucide-react";

import { fetchRecordFile } from "@/lib/api";
import { findRecordInBootstrap } from "@/lib/record-summary";
import { useViewer } from "@/lib/viewer-context";
import { Badge } from "@/components/ui/badge";
import { Button } from "@/components/ui/button";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Skeleton } from "@/components/ui/skeleton";
import { Slider } from "@/components/ui/slider";
import type { UrdfJoint } from "@/components/inspector/JointSlider";

type InspectPanelProps = {
  urdfSpec: { joints: UrdfJoint[] } | null;
  jointValues: Map<string, number>;
  onJointChange: (name: string, value: number) => void;
  onResetAll: () => void;
};

function Field({ label, value }: { label: string; value: string | null | undefined }): JSX.Element {
  return (
    <div className="bg-white px-2.5 py-[7px]">
      <dt className="text-[10px] font-medium uppercase tracking-[0.04em] text-[#aaa]">{label}</dt>
      <dd className="mt-px whitespace-normal break-words text-[12px] text-[#1e1e1e]">{value || "--"}</dd>
    </div>
  );
}

function isMovableJoint(joint: UrdfJoint): boolean {
  return joint.type !== "fixed";
}

function jointRange(joint: UrdfJoint): [number, number] {
  if (joint.type === "continuous") {
    return [-Math.PI, Math.PI];
  }
  return [joint.limit?.lower ?? 0, joint.limit?.upper ?? 0];
}

function formatJointValue(joint: UrdfJoint, value: number): string {
  if (joint.type === "revolute" || joint.type === "continuous") {
    return `${(value * (180 / Math.PI)).toFixed(1)}°`;
  }
  return `${value.toFixed(3)}m`;
}

function jointBadgeVariant(joint: UrdfJoint): "secondary" | "success" {
  return isMovableJoint(joint) ? "success" : "secondary";
}

function SectionHeader({
  eyebrow,
  title,
  meta,
  action,
}: {
  eyebrow: string;
  title: string;
  meta?: string;
  action?: JSX.Element;
}): JSX.Element {
  return (
    <div className="flex items-start justify-between gap-3">
      <div className="min-w-0">
        <p className="text-[10px] font-medium uppercase tracking-[0.05em] text-[#aaa]">{eyebrow}</p>
        <h3 className="mt-0.5 text-[13px] font-medium leading-[1.3] text-[#1e1e1e]">{title}</h3>
        {meta ? <p className="mt-0.5 font-mono text-[10px] text-[#999]">{meta}</p> : null}
      </div>
      {action}
    </div>
  );
}

function buildRootLinks(joints: UrdfJoint[]): string[] {
  const parents = new Set<string>();
  const children = new Set<string>();

  for (const joint of joints) {
    parents.add(joint.parent);
    children.add(joint.child);
  }

  const roots = Array.from(parents).filter((name) => !children.has(name));
  if (roots.length > 0) return roots;
  return Array.from(new Set(joints.map((joint) => joint.parent)));
}

type LinkBranchProps = {
  linkName: string;
  jointsByParent: Map<string, UrdfJoint[]>;
  jointValues: Map<string, number>;
  onJointChange: (name: string, value: number) => void;
  seen: Set<string>;
};

function LinkBranch({ linkName, jointsByParent, jointValues, onJointChange, seen }: LinkBranchProps): JSX.Element {
  const childJoints = jointsByParent.get(linkName) ?? [];

  return (
    <div className="space-y-2.5">
      <div className="flex items-center justify-between gap-3 rounded-sm border border-[#e8e8e8] bg-white px-2.5 py-1.5">
        <div className="flex min-w-0 items-center gap-2">
          <span className="size-2.5 shrink-0 rounded-full border border-[#cfcfcf] bg-[#f7f7f7]" />
          <div className="min-w-0">
            <p className="truncate font-mono text-[11px] text-[#1e1e1e]" title={linkName}>
              {linkName}
            </p>
            <p className="mt-0.5 text-[10px] uppercase tracking-[0.04em] text-[#aaa]">Link</p>
          </div>
        </div>
        <span className="shrink-0 font-mono text-[10px] tabular-nums text-[#aaa]">
          {childJoints.length} child{childJoints.length === 1 ? "" : "ren"}
        </span>
      </div>

      {childJoints.length > 0 && (
        <div className="ml-[5px] space-y-3 border-l border-[#ececec] pl-5">
          {childJoints.map((joint) => {
            const nextSeen = new Set(seen);
            const canDescend = !seen.has(joint.child);
            nextSeen.add(joint.child);
            const [min, max] = jointRange(joint);
            const value = jointValues.get(joint.name) ?? 0;

            return (
              <div key={joint.name} className="relative space-y-2.5">
                <div className="absolute -left-5 top-4 h-px w-4 bg-[#ececec]" />
                <div className="rounded-sm border border-[#e8e8e8] bg-[#fcfcfc] px-2.5 py-2.5">
                  <div className="flex items-start justify-between gap-2">
                    <div className="min-w-0">
                      <div className="flex flex-wrap items-center gap-x-2 gap-y-1">
                        <p className="truncate font-mono text-[11px] text-[#1e1e1e]" title={joint.name}>
                          {joint.name}
                        </p>
                        <span className="text-[10px] text-[#bbb]">to</span>
                        <p className="truncate font-mono text-[11px] text-[#666]" title={joint.child}>
                          {joint.child}
                        </p>
                      </div>
                      <div className="mt-1 flex flex-wrap items-center gap-x-3 gap-y-1 text-[10px] text-[#999]">
                        <span>
                          Parent <span className="font-mono text-[#777]">{joint.parent}</span>
                        </span>
                        <span>
                          Child <span className="font-mono text-[#777]">{joint.child}</span>
                        </span>
                      </div>
                    </div>
                    <Badge variant={jointBadgeVariant(joint)}>{joint.type}</Badge>
                  </div>

                  {isMovableJoint(joint) ? (
                    <div className="mt-2.5 space-y-2">
                      <Slider
                        min={min}
                        max={max}
                        step={0.01}
                        value={[value]}
                        onValueChange={([nextValue]) => onJointChange(joint.name, nextValue)}
                      />
                      <div className="flex justify-between font-mono text-[10px] text-[#bbb]">
                        <span>{formatJointValue(joint, min)}</span>
                        <span className="text-[#666]">{formatJointValue(joint, value)}</span>
                        <span>{formatJointValue(joint, max)}</span>
                      </div>
                    </div>
                  ) : null}
                </div>

                {canDescend ? (
                  <LinkBranch
                    linkName={joint.child}
                    jointsByParent={jointsByParent}
                    jointValues={jointValues}
                    onJointChange={onJointChange}
                    seen={nextSeen}
                  />
                ) : (
                  <div className="ml-2 rounded-sm border border-dashed border-[#e4e4e4] bg-white px-2.5 py-2 text-[10px] text-[#999]">
                    Cycle detected at {joint.child}
                  </div>
                )}
              </div>
            );
          })}
        </div>
      )}
    </div>
  );
}

export function InspectPanel({
  urdfSpec,
  jointValues,
  onJointChange,
  onResetAll,
}: InspectPanelProps): JSX.Element {
  const { bootstrap, selectedRecordId } = useViewer();
  const [promptText, setPromptText] = useState<string | null>(null);
  const [loadingPrompt, setLoadingPrompt] = useState(false);

  const record = selectedRecordId ? findRecordInBootstrap(bootstrap, selectedRecordId) : null;
  const joints = urdfSpec?.joints ?? [];
  const movableJointCount = joints.filter(isMovableJoint).length;

  const { rootLinks, jointsByParent } = useMemo(() => {
    const nextMap = new Map<string, UrdfJoint[]>();
    for (const joint of joints) {
      const current = nextMap.get(joint.parent) ?? [];
      current.push(joint);
      nextMap.set(joint.parent, current);
    }
    return {
      rootLinks: buildRootLinks(joints),
      jointsByParent: nextMap,
    };
  }, [joints]);

  useEffect(() => {
    if (!selectedRecordId || !record) {
      setPromptText(null);
      setLoadingPrompt(false);
      return;
    }

    let cancelled = false;
    setLoadingPrompt(true);

    fetchRecordFile(selectedRecordId, "prompt.txt")
      .then((text) => {
        if (!cancelled) setPromptText(text.trim() || null);
      })
      .catch(() => {
        if (!cancelled) setPromptText(null);
      })
      .finally(() => {
        if (!cancelled) setLoadingPrompt(false);
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
      <div className="space-y-4 pb-2">
        <section className="space-y-1.5">
          <SectionHeader
            eyebrow="Inspect"
            title="Prompt"
            meta="Source prompt used to generate the current record"
          />
          <div className="rounded-sm border border-[#e8e8e8] bg-white px-3 py-3 shadow-[0_1px_0_rgba(0,0,0,0.02)]">
            {loadingPrompt ? (
              <div className="space-y-1.5">
                <Skeleton className="h-3 w-full" />
                <Skeleton className="h-3 w-[92%]" />
                <Skeleton className="h-3 w-[85%]" />
              </div>
            ) : (
              <p className="whitespace-pre-wrap text-[12px] leading-[1.55] text-[#1e1e1e]">
                {promptText ?? record.prompt_preview ?? "--"}
              </p>
            )}
          </div>
        </section>

        <section className="space-y-1.5">
          <SectionHeader
            eyebrow="Record"
            title="Generation Context"
            meta="Primary metadata used to identify and compare outputs"
          />
          <dl className="grid grid-cols-3 gap-px overflow-hidden rounded-sm border border-[#e8e8e8] bg-[#e8e8e8]">
            <Field label="Provider" value={record.provider} />
            <Field label="Model" value={record.model_id} />
            <Field label="Category" value={record.category_slug} />
          </dl>
        </section>

        <section className="space-y-2">
          <SectionHeader
            eyebrow="Mechanism"
            title="Kinematic Tree"
            meta={`${joints.length} joint${joints.length === 1 ? "" : "s"} total, ${movableJointCount} movable`}
            action={
              <Button
                variant="ghost"
                size="sm"
                onClick={onResetAll}
                className="mt-0.5 h-6 gap-1 px-2 text-[11px] text-[#999] hover:text-[#666]"
              >
                <RotateCcw className="size-3" />
                Reset
              </Button>
            }
          />

          {joints.length === 0 ? (
            <div className="flex h-32 items-center justify-center rounded-sm border border-dashed border-[#e4e4e4] bg-white">
              <p className="text-[11px] text-[#bbb]">No joints detected</p>
            </div>
          ) : (
            <div className="space-y-4 rounded-sm border border-[#ececec] bg-[#fcfcfc] p-3">
              {rootLinks.map((rootLink) => (
                <LinkBranch
                  key={rootLink}
                  linkName={rootLink}
                  jointsByParent={jointsByParent}
                  jointValues={jointValues}
                  onJointChange={onJointChange}
                  seen={new Set([rootLink])}
                />
              ))}
            </div>
          )}
        </section>
      </div>
    </ScrollArea>
  );
}
