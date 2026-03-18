import { useState, useCallback, useRef, type JSX } from "react";
import { ChevronLeft } from "lucide-react";
import type { PanelImperativeHandle, PanelSize } from "react-resizable-panels";
import type * as THREE from "three";

import { ViewerProvider } from "@/lib/viewer-context";
import {
  ResizablePanelGroup,
  ResizablePanel,
  ResizableHandle,
} from "@/components/ui/resizable";
import { TooltipProvider } from "@/components/ui/tooltip";
import { AppHeader } from "@/components/layout/AppHeader";
import { Sidebar } from "@/components/layout/Sidebar";
import { Inspector } from "@/components/layout/Inspector";
import { ViewportPanel } from "@/components/layout/ViewportPanel";
import { InspectorTabs } from "@/components/inspector/InspectorTabs";
import type { RenderOptions as InspectorRenderOptions } from "@/components/inspector/RenderOptionsPanel";
import { RecordBrowser } from "@/components/browser/RecordBrowser";
import { Button } from "@/components/ui/button";
import { useRenderOptions } from "@/components/viewer3d/useRenderOptions";
import { useJointController } from "@/components/viewer3d/useJointController";
import type { UrdfSpec } from "@/components/viewer3d/urdf-parser";

export default function App(): JSX.Element {
  const { options: renderOptions, setOption: setRenderOption } = useRenderOptions();

  const [urdfSpec, setUrdfSpec] = useState<UrdfSpec | null>(null);
  const [jointNodes, setJointNodes] = useState<Map<string, THREE.Object3D> | null>(null);
  const [inspectorCollapsed, setInspectorCollapsed] = useState(false);
  const inspectorPanelRef = useRef<PanelImperativeHandle | null>(null);

  const { jointValues, setJointValue, resetAll } = useJointController(jointNodes, urdfSpec);

  const handleUrdfSpecChange = useCallback(
    (spec: UrdfSpec | null, nodes: Map<string, THREE.Object3D> | null) => {
      setUrdfSpec(spec);
      setJointNodes(nodes);
    },
    [],
  );

  const handleRenderOptionChange = useCallback(
    <K extends keyof InspectorRenderOptions>(key: K, value: InspectorRenderOptions[K]) => {
      if (key === "environmentLighting") {
        setRenderOption("envLighting", value);
        return;
      }

      setRenderOption(key, value);
    },
    [setRenderOption],
  );

  const handleInspectorResize = useCallback((size: PanelSize) => {
    setInspectorCollapsed(size.inPixels <= 24);
  }, []);

  const handleInspectorExpand = useCallback(() => {
    inspectorPanelRef.current?.expand();
  }, []);

  const inspectorRenderOptions = {
    showEdges: renderOptions.showEdges,
    showGrid: renderOptions.showGrid,
    showCollisions: renderOptions.showCollisions,
    doubleSided: renderOptions.doubleSided,
    environmentLighting: renderOptions.envLighting,
  };

  const inspectorUrdfSpec = urdfSpec ? { joints: urdfSpec.joints } : null;

  return (
    <ViewerProvider>
      <TooltipProvider>
        <div className="flex h-screen flex-col bg-[#f3f3f3]">
          <AppHeader />
          <div className="relative min-h-0 flex-1">
            <ResizablePanelGroup orientation="horizontal" className="h-full">
              <ResizablePanel defaultSize="20%" minSize="15%" maxSize="30%" className="min-w-0">
                <Sidebar>
                  <RecordBrowser />
                </Sidebar>
              </ResizablePanel>
              <ResizableHandle withHandle />
              <ResizablePanel defaultSize="55%" className="min-w-0">
                <ViewportPanel
                  renderOptions={renderOptions}
                  onUrdfSpecChange={handleUrdfSpecChange}
                />
              </ResizablePanel>
              <ResizableHandle withHandle />
              <ResizablePanel
                defaultSize="25%"
                minSize="15%"
                maxSize="35%"
                className="min-w-0"
                collapsible
                panelRef={inspectorPanelRef}
                onResize={handleInspectorResize}
              >
                <Inspector>
                  <InspectorTabs
                    urdfSpec={inspectorUrdfSpec}
                    jointValues={jointValues}
                    onJointChange={setJointValue}
                    onResetAll={resetAll}
                    renderOptions={inspectorRenderOptions}
                    onRenderOptionChange={handleRenderOptionChange}
                  />
                </Inspector>
              </ResizablePanel>
            </ResizablePanelGroup>
            {inspectorCollapsed ? (
              <Button
                type="button"
                variant="outline"
                size="sm"
                onClick={handleInspectorExpand}
                aria-label="Expand inspector panel"
                className="absolute right-3 top-1/2 z-20 h-8 w-8 -translate-y-1/2 rounded-full border-[#d8d8d8] bg-white/95 p-0 text-[#666] shadow-sm backdrop-blur hover:bg-white hover:text-[#1e1e1e]"
              >
                <ChevronLeft className="size-4" />
              </Button>
            ) : null}
          </div>
        </div>
      </TooltipProvider>
    </ViewerProvider>
  );
}
