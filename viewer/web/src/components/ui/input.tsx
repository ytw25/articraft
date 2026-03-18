import * as React from "react"

import { cn } from "@/lib/utils"

function Input({ className, type, ...props }: React.ComponentProps<"input">) {
  return (
    <input
      type={type}
      data-slot="input"
      className={cn(
        "h-8 w-full min-w-0 rounded-sm border border-[#e4e4e4] bg-white px-2.5 py-1.5 text-[12px] text-[#1e1e1e] shadow-none transition-[border-color] outline-none placeholder:text-[#bbb] disabled:pointer-events-none disabled:opacity-50",
        "focus-visible:border-[#007acc]",
        "aria-invalid:border-[#dc2626]",
        className
      )}
      {...props}
    />
  )
}

export { Input }
