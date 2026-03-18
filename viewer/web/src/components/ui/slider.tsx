import * as React from "react";
import { Slider as SliderPrimitive } from "radix-ui";

import { cn } from "@/lib/utils";

export function Slider({
  className,
  defaultValue,
  value,
  min = 0,
  max = 100,
  ...props
}: React.ComponentProps<typeof SliderPrimitive.Root>): React.JSX.Element {
  return (
    <SliderPrimitive.Root
      data-slot="slider"
      defaultValue={defaultValue}
      value={value}
      min={min}
      max={max}
      className={cn("relative flex w-full touch-none select-none items-center", className)}
      {...props}
    >
      <SliderPrimitive.Track className="relative h-[4px] w-full grow overflow-hidden rounded-full bg-[#e4e4e4]">
        <SliderPrimitive.Range className="absolute h-full bg-[#007acc]" />
      </SliderPrimitive.Track>
      <SliderPrimitive.Thumb className="block size-[13px] rounded-full border border-[#ccc] bg-white transition-colors hover:border-[#007acc] focus-visible:outline-none focus-visible:ring-1 focus-visible:ring-[#007acc]" />
    </SliderPrimitive.Root>
  );
}
