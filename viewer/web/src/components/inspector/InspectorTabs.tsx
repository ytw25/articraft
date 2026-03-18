import { useState, type JSX } from "react";

import { useViewer } from "@/lib/viewer-context";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { InspectPanel } from "@/components/inspector/InspectPanel";
import { MetadataPanel } from "@/components/inspector/MetadataPanel";
import {
  RenderOptionsPanel,
  defaultRenderOptions,
  type RenderOptions,
} from "@/components/inspector/RenderOptionsPanel";
import { CodePanel } from "@/components/inspector/CodePanel";
import type { UrdfJoint } from "@/components/inspector/JointSlider";

type InspectorTabsProps = {
  urdfSpec?: { joints: UrdfJoint[] } | null;
  jointValues?: Map<string, number>;
  onJointChange?: (name: string, value: number) => void;
  onResetAll?: () => void;
  renderOptions?: RenderOptions;
  onRenderOptionChange?: <K extends keyof RenderOptions>(key: K, value: RenderOptions[K]) => void;
};

export function InspectorTabs({
  urdfSpec = null,
  jointValues = new Map(),
  onJointChange = () => {},
  onResetAll = () => {},
  renderOptions,
  onRenderOptionChange,
}: InspectorTabsProps): JSX.Element {
  const { selectedRecordId } = useViewer();

  const [localOptions, setLocalOptions] = useState<RenderOptions>(defaultRenderOptions);
  const activeOptions = renderOptions ?? localOptions;
  const handleOptionChange = onRenderOptionChange ?? (<K extends keyof RenderOptions>(key: K, value: RenderOptions[K]) => {
    setLocalOptions((prev) => ({ ...prev, [key]: value }));
  });

  const tabs = ["inspect", "render", "code", "info"] as const;

  return (
    <Tabs defaultValue="inspect" className="relative flex h-full flex-col gap-0">
      {/* Tab bar — underline style */}
      <div className="flex border-b border-[#e8e8e8] px-3">
        <TabsList className="flex h-auto gap-0 rounded-none border-none bg-transparent p-0">
          {tabs.map((tab) => (
            <TabsTrigger
              key={tab}
              value={tab}
              className="rounded-none border-b-[1.5px] border-transparent px-2.5 pb-2 pt-2.5 text-[11px] font-medium capitalize text-[#999] transition-colors hover:text-[#666] data-[state=active]:border-b-[#1e1e1e] data-[state=active]:bg-transparent data-[state=active]:text-[#1e1e1e] data-[state=active]:shadow-none"
            >
              {tab.charAt(0).toUpperCase() + tab.slice(1)}
            </TabsTrigger>
          ))}
        </TabsList>
      </div>

      <TabsContent value="inspect" className="min-h-0 flex-1 overflow-hidden px-2.5 pb-2.5 pt-2">
        <InspectPanel
          urdfSpec={urdfSpec}
          jointValues={jointValues}
          onJointChange={onJointChange}
          onResetAll={onResetAll}
        />
      </TabsContent>

      <TabsContent value="render" className="min-h-0 flex-1 overflow-hidden px-2.5 pb-2.5 pt-2">
        <RenderOptionsPanel options={activeOptions} onOptionChange={handleOptionChange} />
      </TabsContent>

      <TabsContent value="code" className="min-h-0 flex-1 overflow-hidden px-2.5 pb-2.5 pt-2">
        <CodePanel />
      </TabsContent>

      <TabsContent value="info" className="min-h-0 flex-1 overflow-hidden px-2.5 pb-2.5 pt-2">
        <MetadataPanel />
      </TabsContent>

      {!selectedRecordId && (
        <div className="absolute inset-0 flex items-center justify-center bg-[#f8f8f8]/90">
          <div className="px-4 py-2.5 text-center">
            <p className="text-[12px] text-[#999]">No record selected</p>
          </div>
        </div>
      )}
    </Tabs>
  );
}
