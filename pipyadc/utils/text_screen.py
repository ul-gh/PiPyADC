class TextScreen():
    """Output arbitrary number of lines of text, while re-using the screen
    area previously written to.

    For this, text is put in a buffer using the put() method, and output is
    written to the screen all at once when the refresh() method is called.

    Ulrich Lukas 2022-08-27
    """
    def __init__(self):
        self._lines_in_buffer = 0
        self._lines_printed = 0
        self._text_buffer = ""

    def put(self, text):
        """Put lines of text in buffer, but do not output anything.

        Write output all at once, clearing the previous screen contents,
        when refresh() is later called.
        """
        self._lines_in_buffer += 1 + text.count("\n")
        self._text_buffer += text.replace("\n", "\x1B[0K\n") + "\x1B[0K\n"

    def refresh(self):
        """Write output from buffer to screen, clearing previous content
        """
        clear_code = f"\x1B[{self._lines_printed}F" if self._lines_printed else ""
        print(f"{clear_code}{self._text_buffer}", end="")
        self._lines_printed = self._lines_in_buffer
        self._text_buffer = ""
        self._lines_in_buffer = 0


