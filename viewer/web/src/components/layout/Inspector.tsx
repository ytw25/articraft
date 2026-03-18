import { type JSX, type ReactNode } from "react";

import { useViewer } from "@/lib/viewer-context";

interface InspectorProps {
  children: ReactNode;
}

export function Inspector({ children }: InspectorProps): JSX.Element {
  const { inspectorOpen } = useViewer();

  if (!inspectorOpen) {
    return <aside className="h-full min-w-0 bg-[#f8f8f8]" />;
  }

  return (
    <aside className="flex h-full min-w-0 flex-col overflow-hidden border-l border-[#e4e4e4] bg-[#f8f8f8]">
      {children}
    </aside>
  );
}
