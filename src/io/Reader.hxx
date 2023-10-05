// SPDX-License-Identifier: BSD-2-Clause
// author: Max Kellermann <max.kellermann@gmail.com>

#pragma once

#include <cstddef>
#include <type_traits>

/**
 * An interface that can read bytes from a stream until the stream
 * ends.
 *
 * This interface is simpler and less cumbersome to use than
 * #InputStream.
 */
class Reader {
public:
	Reader() = default;
	Reader(const Reader &) = delete;

	virtual ~Reader() noexcept = default;

	/**
	 * Read data from the stream.
	 *
	 * @return the number of bytes read into the given buffer or 0
	 * on end-of-stream
	 */
	[[gnu::nonnull]]
	virtual std::size_t Read(void *data, std::size_t size) = 0;

	template<typename T>
	requires std::is_standard_layout_v<T> && std::is_trivially_copyable_v<T>
	void ReadT(T &dest) {
		// TODO check return value
		Read(&dest, sizeof(dest));
	}
};
