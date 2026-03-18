import type { JSX } from "react";

import { Label } from "@/components/ui/label";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Switch } from "@/components/ui/switch";

export type RenderOptions = {
  showEdges: boolean;
  showGrid: boolean;
  showCollisions: boolean;
  doubleSided: boolean;
  environmentLighting: boolean;
};

export const defaultRenderOptions: RenderOptions = {
  showEdges: false,
  showGrid: true,
  showCollisions: false,
  doubleSided: false,
  environmentLighting: true,
};

type RenderOptionsPanelProps = {
  options: RenderOptions;
  onOptionChange: <K extends keyof RenderOptions>(key: K, value: RenderOptions[K]) => void;
};

type OptionRow = {
  key: keyof RenderOptions;
  label: string;
  description: string;
};

const optionRows: OptionRow[] = [
  {
    key: "showEdges",
    label: "Edges",
    description: "Wireframe overlay on meshes",
  },
  {
    key: "showGrid",
    label: "Grid",
    description: "Ground-plane reference grid",
  },
  {
    key: "showCollisions",
    label: "Collisions",
    description: "Semi-transparent collision geometry",
  },
  {
    key: "doubleSided",
    label: "Double-sided",
    description: "Render front & back faces",
  },
  {
    key: "environmentLighting",
    label: "Environment",
    description: "HDR environment lighting",
  },
];

export function RenderOptionsPanel({ options, onOptionChange }: RenderOptionsPanelProps): JSX.Element {
  return (
    <ScrollArea className="h-full">
      <div className="pb-1">
        {optionRows.map((row) => (
          <div
            key={row.key}
            className="flex items-center justify-between gap-3 py-2"
          >
            <div className="min-w-0">
              <Label htmlFor={`render-${row.key}`} className="text-[12px] text-[#1e1e1e]">
                {row.label}
              </Label>
              <p className="text-[10px] leading-[1.3] text-[#aaa]">{row.description}</p>
            </div>
            <Switch
              id={`render-${row.key}`}
              checked={options[row.key]}
              onCheckedChange={(checked: boolean) => onOptionChange(row.key, checked)}
            />
          </div>
        ))}
      </div>
    </ScrollArea>
  );
}
